#include "aos/events/logging/logfile_utils.h"

#include <fcntl.h>
#include <limits.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>

#include <vector>

#include "aos/configuration.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/flatbuffer_merge.h"
#include "aos/util/file.h"
#include "flatbuffers/flatbuffers.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_int32(flush_size, 1000000,
             "Number of outstanding bytes to allow before flushing to disk.");

namespace aos {
namespace logger {

namespace chrono = std::chrono;

DetachedBufferWriter::DetachedBufferWriter(std::string_view filename)
    : filename_(filename) {
  util::MkdirP(filename, 0777);
  fd_ = open(std::string(filename).c_str(),
             O_RDWR | O_CLOEXEC | O_CREAT | O_EXCL, 0774);
  VLOG(1) << "Opened " << filename << " for writing";
  PCHECK(fd_ != -1) << ": Failed to open " << filename << " for writing";
}

DetachedBufferWriter::~DetachedBufferWriter() {
  Flush();
  PLOG_IF(ERROR, close(fd_) == -1) << " Failed to close logfile";
}

void DetachedBufferWriter::QueueSizedFlatbuffer(
    flatbuffers::FlatBufferBuilder *fbb) {
  QueueSizedFlatbuffer(fbb->Release());
}

void DetachedBufferWriter::WriteSizedFlatbuffer(
    absl::Span<const uint8_t> span) {
  // Cheat aggressively...  Write out the queued up data, and then write this
  // data once without buffering.  It is hard to make a DetachedBuffer out of
  // this data, and we don't want to worry about lifetimes.
  Flush();
  iovec_.clear();
  iovec_.reserve(1);

  struct iovec n;
  n.iov_base = const_cast<uint8_t *>(span.data());
  n.iov_len = span.size();
  iovec_.emplace_back(n);

  const ssize_t written = writev(fd_, iovec_.data(), iovec_.size());

  PCHECK(written == static_cast<ssize_t>(n.iov_len))
      << ": Wrote " << written << " expected " << n.iov_len;
}

void DetachedBufferWriter::QueueSizedFlatbuffer(
    flatbuffers::DetachedBuffer &&buffer) {
  queued_size_ += buffer.size();
  queue_.emplace_back(std::move(buffer));

  // Flush if we are at the max number of iovs per writev, or have written
  // enough data.  Otherwise writev will fail with an invalid argument.
  if (queued_size_ > static_cast<size_t>(FLAGS_flush_size) ||
      queue_.size() == IOV_MAX) {
    Flush();
  }
}

void DetachedBufferWriter::Flush() {
  if (queue_.size() == 0u) {
    return;
  }
  iovec_.clear();
  iovec_.reserve(queue_.size());
  size_t counted_size = 0;
  for (size_t i = 0; i < queue_.size(); ++i) {
    struct iovec n;
    n.iov_base = queue_[i].data();
    n.iov_len = queue_[i].size();
    counted_size += n.iov_len;
    iovec_.emplace_back(std::move(n));
  }
  CHECK_EQ(counted_size, queued_size_);
  const ssize_t written = writev(fd_, iovec_.data(), iovec_.size());

  PCHECK(written == static_cast<ssize_t>(queued_size_))
      << ": Wrote " << written << " expected " << queued_size_;

  queued_size_ = 0;
  queue_.clear();
  // TODO(austin): Handle partial writes in some way other than crashing...
}

flatbuffers::Offset<MessageHeader> PackMessage(
    flatbuffers::FlatBufferBuilder *fbb, const Context &context,
    int channel_index, LogType log_type) {
  flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset;

  switch (log_type) {
    case LogType::kLogMessage:
    case LogType::kLogMessageAndDeliveryTime:
    case LogType::kLogRemoteMessage:
      data_offset =
          fbb->CreateVector(static_cast<uint8_t *>(context.data), context.size);
      break;

    case LogType::kLogDeliveryTimeOnly:
      break;
  }

  MessageHeader::Builder message_header_builder(*fbb);
  message_header_builder.add_channel_index(channel_index);

  switch (log_type) {
    case LogType::kLogRemoteMessage:
      message_header_builder.add_queue_index(context.remote_queue_index);
      message_header_builder.add_monotonic_sent_time(
          context.monotonic_remote_time.time_since_epoch().count());
      message_header_builder.add_realtime_sent_time(
          context.realtime_remote_time.time_since_epoch().count());
      break;

    case LogType::kLogMessage:
    case LogType::kLogMessageAndDeliveryTime:
    case LogType::kLogDeliveryTimeOnly:
      message_header_builder.add_queue_index(context.queue_index);
      message_header_builder.add_monotonic_sent_time(
          context.monotonic_event_time.time_since_epoch().count());
      message_header_builder.add_realtime_sent_time(
          context.realtime_event_time.time_since_epoch().count());
      break;
  }

  switch (log_type) {
    case LogType::kLogMessage:
    case LogType::kLogRemoteMessage:
      message_header_builder.add_data(data_offset);
      break;

    case LogType::kLogMessageAndDeliveryTime:
      message_header_builder.add_data(data_offset);
      [[fallthrough]];

    case LogType::kLogDeliveryTimeOnly:
      message_header_builder.add_monotonic_remote_time(
          context.monotonic_remote_time.time_since_epoch().count());
      message_header_builder.add_realtime_remote_time(
          context.realtime_remote_time.time_since_epoch().count());
      message_header_builder.add_remote_queue_index(context.remote_queue_index);
      break;
  }

  return message_header_builder.Finish();
}

SpanReader::SpanReader(std::string_view filename)
    : filename_(filename),
      fd_(open(std::string(filename).c_str(), O_RDONLY | O_CLOEXEC)) {
  PCHECK(fd_ != -1) << ": Failed to open " << filename;
}

absl::Span<const uint8_t> SpanReader::ReadMessage() {
  // Make sure we have enough for the size.
  if (data_.size() - consumed_data_ < sizeof(flatbuffers::uoffset_t)) {
    if (!ReadBlock()) {
      return absl::Span<const uint8_t>();
    }
  }

  // Now make sure we have enough for the message.
  const size_t data_size =
      flatbuffers::GetPrefixedSize(data_.data() + consumed_data_) +
      sizeof(flatbuffers::uoffset_t);
  while (data_.size() < consumed_data_ + data_size) {
    if (!ReadBlock()) {
      return absl::Span<const uint8_t>();
    }
  }

  // And return it, consuming the data.
  const uint8_t *data_ptr = data_.data() + consumed_data_;

  consumed_data_ += data_size;

  return absl::Span<const uint8_t>(data_ptr, data_size);
}

bool SpanReader::MessageAvailable() {
  // Are we big enough to read the size?
  if (data_.size() - consumed_data_ < sizeof(flatbuffers::uoffset_t)) {
    return false;
  }

  // Then, are we big enough to read the full message?
  const size_t data_size =
      flatbuffers::GetPrefixedSize(data_.data() + consumed_data_) +
      sizeof(flatbuffers::uoffset_t);
  if (data_.size() < consumed_data_ + data_size) {
    return false;
  }

  return true;
}

bool SpanReader::ReadBlock() {
  if (end_of_file_) {
    return false;
  }

  // Appends 256k.  This is enough that the read call is efficient.  We don't
  // want to spend too much time reading small chunks because the syscalls for
  // that will be expensive.
  constexpr size_t kReadSize = 256 * 1024;

  // Strip off any unused data at the front.
  if (consumed_data_ != 0) {
    data_.erase(data_.begin(), data_.begin() + consumed_data_);
    consumed_data_ = 0;
  }

  const size_t starting_size = data_.size();

  // This should automatically grow the backing store.  It won't shrink if we
  // get a small chunk later.  This reduces allocations when we want to append
  // more data.
  data_.resize(data_.size() + kReadSize);

  ssize_t count = read(fd_, &data_[starting_size], kReadSize);
  data_.resize(starting_size + std::max(count, static_cast<ssize_t>(0)));
  if (count == 0) {
    end_of_file_ = true;
    return false;
  }
  PCHECK(count > 0);

  return true;
}

FlatbufferVector<LogFileHeader> ReadHeader(std::string_view filename) {
  SpanReader span_reader(filename);
  // Make sure we have enough to read the size.
  absl::Span<const uint8_t> config_data = span_reader.ReadMessage();

  // Make sure something was read.
  CHECK(config_data != absl::Span<const uint8_t>());

  // And copy the config so we have it forever.
  std::vector<uint8_t> data(
      config_data.begin() + sizeof(flatbuffers::uoffset_t), config_data.end());
  return FlatbufferVector<LogFileHeader>(std::move(data));
}

MessageReader::MessageReader(std::string_view filename)
    : span_reader_(filename) {
  // Make sure we have enough to read the size.
  absl::Span<const uint8_t> config_data = span_reader_.ReadMessage();

  // Make sure something was read.
  CHECK(config_data != absl::Span<const uint8_t>());

  // And copy the config so we have it forever.
  configuration_ = std::vector<uint8_t>(config_data.begin(), config_data.end());

  max_out_of_order_duration_ = std::chrono::nanoseconds(
      flatbuffers::GetSizePrefixedRoot<LogFileHeader>(configuration_.data())
          ->max_out_of_order_duration());
}

std::optional<FlatbufferVector<MessageHeader>> MessageReader::ReadMessage() {
  absl::Span<const uint8_t> msg_data = span_reader_.ReadMessage();
  if (msg_data == absl::Span<const uint8_t>()) {
    return std::nullopt;
  }

  FlatbufferVector<MessageHeader> result{std::vector<uint8_t>(
      msg_data.begin() + sizeof(flatbuffers::uoffset_t), msg_data.end())};

  const monotonic_clock::time_point timestamp = monotonic_clock::time_point(
      chrono::nanoseconds(result.message().monotonic_sent_time()));

  newest_timestamp_ = std::max(newest_timestamp_, timestamp);
  VLOG(1) << "Read from " << filename().substr(130) << " data "
          << FlatbufferToJson(result);
  return std::move(result);
}

SplitMessageReader::SplitMessageReader(
    const std::vector<std::string> &filenames)
    : filenames_(filenames),
      log_file_header_(FlatbufferDetachedBuffer<LogFileHeader>::Empty()) {
  CHECK(NextLogFile()) << ": filenames is empty.  Need files to read.";

  // Grab any log file header.  They should all match (and we will check as we
  // open more of them).
  log_file_header_ = CopyFlatBuffer(message_reader_->log_file_header());

  // Setup per channel state.
  channels_.resize(configuration()->channels()->size());
  for (ChannelData &channel_data : channels_) {
    channel_data.data.split_reader = this;
    // Build up the timestamp list.
    if (configuration::MultiNode(configuration())) {
      channel_data.timestamps.resize(configuration()->nodes()->size());
      for (MessageHeaderQueue &queue : channel_data.timestamps) {
        queue.timestamps = true;
        queue.split_reader = this;
      }
    }
  }

  // Build up channels_to_write_ as an optimization to make it fast to figure
  // out which datastructure to place any new data from a channel on.
  for (const Channel *channel : *configuration()->channels()) {
    // This is the main case.  We will only see data on this node.
    if (configuration::ChannelIsSendableOnNode(channel, node())) {
      channels_to_write_.emplace_back(
          &channels_[channels_to_write_.size()].data);
    } else
        // If we can't send, but can receive, we should be able to see
        // timestamps here.
        if (configuration::ChannelIsReadableOnNode(channel, node())) {
      channels_to_write_.emplace_back(
          &(channels_[channels_to_write_.size()]
                .timestamps[configuration::GetNodeIndex(configuration(),
                                                        node())]));
    } else {
      channels_to_write_.emplace_back(nullptr);
    }
  }
}

bool SplitMessageReader::NextLogFile() {
  if (next_filename_index_ == filenames_.size()) {
    return false;
  }
  message_reader_ =
      std::make_unique<MessageReader>(filenames_[next_filename_index_]);

  // We can't support the config diverging between two log file headers.  See if
  // they are the same.
  if (next_filename_index_ != 0) {
    CHECK(CompareFlatBuffer(&log_file_header_.message(),
                            message_reader_->log_file_header()))
        << ": Header is different between log file chunks "
        << filenames_[next_filename_index_] << " and "
        << filenames_[next_filename_index_ - 1] << ", this is not supported.";
  }

  ++next_filename_index_;
  return true;
}

bool SplitMessageReader::QueueMessages(
    monotonic_clock::time_point oldest_message_time) {
  // TODO(austin): Once we are happy that everything works, read a 256kb chunk
  // to reduce the need to re-heap down below.
  while (true) {
    // Don't queue if we have enough data already.
    // When a log file starts, there should be a message from each channel.
    // Those messages might be very old. Make sure to read a chunk past the
    // starting time.
    if (queued_messages_ > 0 &&
        message_reader_->queue_data_time() > oldest_message_time) {
      return true;
    }

    if (std::optional<FlatbufferVector<MessageHeader>> msg =
            message_reader_->ReadMessage()) {
      const MessageHeader &header = msg.value().message();

      const int channel_index = header.channel_index();
      channels_to_write_[channel_index]->emplace_back(std::move(msg.value()));

      ++queued_messages_;
    } else {
      if (!NextLogFile()) {
        return false;
      }
    }
  }
}

void SplitMessageReader::SetTimestampMerger(TimestampMerger *timestamp_merger,
                                            int channel_index,
                                            const Node *target_node) {
  const Node *reinterpreted_target_node =
      configuration::GetNodeOrDie(configuration(), target_node);
  const Channel *const channel =
      configuration()->channels()->Get(channel_index);

  MessageHeaderQueue *message_header_queue = nullptr;

  // Figure out if this log file is from our point of view, or the other node's
  // point of view.
  if (node() == reinterpreted_target_node) {
    if (channels_to_write_[channel_index] != nullptr) {
      // We already have deduced which is the right channel.  Use
      // channels_to_write_ here.
      message_header_queue = channels_to_write_[channel_index];
    } else {
      // This means this is data from another node, and will be ignored.
    }
  } else {
    // We are replaying from another node's point of view.  The only interesting
    // data is data that is forwarded to our node, ie was sent on the other
    // node.
    if (configuration::ChannelIsSendableOnNode(channel, node())) {
      // Data from another node.
      message_header_queue = &(channels_[channel_index].data);
    } else {
      // This is either not sendable on the other node, or is a timestamp and
      // therefore not interesting.
    }
  }

  // If we found one, write it down.  This will be nullptr when there is nothing
  // relevant on this channel on this node for the target node.  In that case,
  // we want to drop the message instead of queueing it.
  if (message_header_queue != nullptr) {
    message_header_queue->timestamp_merger = timestamp_merger;
  }
}

std::tuple<monotonic_clock::time_point, uint32_t,
           FlatbufferVector<MessageHeader>>
SplitMessageReader::PopOldest(int channel_index) {
  CHECK_GT(channels_[channel_index].data.size(), 0u);
  const std::tuple<monotonic_clock::time_point, uint32_t> timestamp =
      channels_[channel_index].data.front_timestamp();
  FlatbufferVector<MessageHeader> front =
      std::move(channels_[channel_index].data.front());
  channels_[channel_index].data.pop_front();
  --queued_messages_;

  return std::make_tuple(std::get<0>(timestamp), std::get<1>(timestamp),
                         std::move(front));
}

std::tuple<monotonic_clock::time_point, uint32_t,
           FlatbufferVector<MessageHeader>>
SplitMessageReader::PopOldest(int channel, int node_index) {
  CHECK_GT(channels_[channel].timestamps[node_index].size(), 0u);
  const std::tuple<monotonic_clock::time_point, uint32_t> timestamp =
      channels_[channel].timestamps[node_index].front_timestamp();
  FlatbufferVector<MessageHeader> front =
      std::move(channels_[channel].timestamps[node_index].front());
  channels_[channel].timestamps[node_index].pop_front();
  --queued_messages_;

  return std::make_tuple(std::get<0>(timestamp), std::get<1>(timestamp),
                         std::move(front));
}

void SplitMessageReader::MessageHeaderQueue::emplace_back(
    FlatbufferVector<MessageHeader> &&msg) {
  CHECK(split_reader != nullptr);

  // If there is no timestamp merger for this queue, nobody is listening.  Drop
  // the message.  This happens when a log file from another node is replayed,
  // and the timestamp mergers down stream just don't care.
  if (timestamp_merger == nullptr) {
    return;
  }

  CHECK(timestamps != msg.message().has_data())
      << ": Got timestamps and data mixed up on a node. "
      << FlatbufferToJson(msg);

  data_.emplace_back(std::move(msg));

  if (data_.size() == 1u) {
    // Yup, new data.  Notify.
    if (timestamps) {
      timestamp_merger->UpdateTimestamp(split_reader, front_timestamp());
    } else {
      timestamp_merger->Update(split_reader, front_timestamp());
    }
  }
}

void SplitMessageReader::MessageHeaderQueue::pop_front() {
  data_.pop_front();
  if (data_.size() != 0u) {
    // Yup, new data.
    if (timestamps) {
      timestamp_merger->UpdateTimestamp(split_reader, front_timestamp());
    } else {
      timestamp_merger->Update(split_reader, front_timestamp());
    }
  }
}

namespace {

bool SplitMessageReaderHeapCompare(
    const std::tuple<monotonic_clock::time_point, uint32_t,
                     SplitMessageReader *>
        first,
    const std::tuple<monotonic_clock::time_point, uint32_t,
                     SplitMessageReader *>
        second) {
  if (std::get<0>(first) > std::get<0>(second)) {
    return true;
  } else if (std::get<0>(first) == std::get<0>(second)) {
    if (std::get<1>(first) > std::get<1>(second)) {
      return true;
    } else if (std::get<1>(first) == std::get<1>(second)) {
      return std::get<2>(first) > std::get<2>(second);
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool ChannelHeapCompare(
    const std::pair<monotonic_clock::time_point, int> first,
    const std::pair<monotonic_clock::time_point, int> second) {
  if (first.first > second.first) {
    return true;
  } else if (first.first == second.first) {
    return first.second > second.second;
  } else {
    return false;
  }
}

}  // namespace

TimestampMerger::TimestampMerger(
    const Configuration *configuration,
    std::vector<SplitMessageReader *> split_message_readers, int channel_index,
    const Node *target_node, ChannelMerger *channel_merger)
    : configuration_(configuration),
      split_message_readers_(std::move(split_message_readers)),
      channel_index_(channel_index),
      node_index_(configuration::MultiNode(configuration)
                      ? configuration::GetNodeIndex(configuration, target_node)
                      : -1),
      channel_merger_(channel_merger) {
  // Tell the readers we care so they know who to notify.
  for (SplitMessageReader *reader : split_message_readers_) {
    reader->SetTimestampMerger(this, channel_index, target_node);
  }

  // And then determine if we need to track timestamps.
  const Channel *channel = configuration->channels()->Get(channel_index);
  if (!configuration::ChannelIsSendableOnNode(channel, target_node) &&
      configuration::ChannelIsReadableOnNode(channel, target_node)) {
    has_timestamps_ = true;
  }
}

void TimestampMerger::PushMessageHeap(
    std::tuple<monotonic_clock::time_point, uint32_t> timestamp,
    SplitMessageReader *split_message_reader) {
  DCHECK(std::find_if(message_heap_.begin(), message_heap_.end(),
                      [split_message_reader](
                          const std::tuple<monotonic_clock::time_point,
                                           uint32_t, SplitMessageReader *>
                              x) {
                        return std::get<2>(x) == split_message_reader;
                      }) == message_heap_.end())
      << ": Pushing message when it is already in the heap.";

  message_heap_.push_back(std::make_tuple(
      std::get<0>(timestamp), std::get<1>(timestamp), split_message_reader));

  std::push_heap(message_heap_.begin(), message_heap_.end(),
                 &SplitMessageReaderHeapCompare);

  // If we are just a data merger, don't wait for timestamps.
  if (!has_timestamps_) {
    channel_merger_->Update(std::get<0>(timestamp), channel_index_);
    pushed_ = true;
  }
}

void TimestampMerger::PushTimestampHeap(
    std::tuple<monotonic_clock::time_point, uint32_t> timestamp,
    SplitMessageReader *split_message_reader) {
  DCHECK(std::find_if(timestamp_heap_.begin(), timestamp_heap_.end(),
                      [split_message_reader](
                          const std::tuple<monotonic_clock::time_point,
                                           uint32_t, SplitMessageReader *>
                              x) {
                        return std::get<2>(x) == split_message_reader;
                      }) == timestamp_heap_.end())
      << ": Pushing timestamp when it is already in the heap.";

  timestamp_heap_.push_back(std::make_tuple(
      std::get<0>(timestamp), std::get<1>(timestamp), split_message_reader));

  std::push_heap(timestamp_heap_.begin(), timestamp_heap_.end(),
                 SplitMessageReaderHeapCompare);

  // If we are a timestamp merger, don't wait for data.  Missing data will be
  // caught at read time.
  if (has_timestamps_) {
    channel_merger_->Update(std::get<0>(timestamp), channel_index_);
    pushed_ = true;
  }
}

std::tuple<monotonic_clock::time_point, uint32_t,
           FlatbufferVector<MessageHeader>>
TimestampMerger::PopMessageHeap() {
  // Pop the oldest message reader pointer off the heap.
  CHECK_GT(message_heap_.size(), 0u);
  std::tuple<monotonic_clock::time_point, uint32_t, SplitMessageReader *>
      oldest_message_reader = message_heap_.front();

  std::pop_heap(message_heap_.begin(), message_heap_.end(),
                &SplitMessageReaderHeapCompare);
  message_heap_.pop_back();

  // Pop the oldest message.  This re-pushes any messages from the reader to the
  // message heap.
  std::tuple<monotonic_clock::time_point, uint32_t,
             FlatbufferVector<MessageHeader>>
      oldest_message =
          std::get<2>(oldest_message_reader)->PopOldest(channel_index_);

  // Confirm that the time and queue_index we have recorded matches.
  CHECK_EQ(std::get<0>(oldest_message), std::get<0>(oldest_message_reader));
  CHECK_EQ(std::get<1>(oldest_message), std::get<1>(oldest_message_reader));

  // Now, keep reading until we have found all duplicates.
  while (message_heap_.size() > 0u) {
    // See if it is a duplicate.
    std::tuple<monotonic_clock::time_point, uint32_t, SplitMessageReader *>
        next_oldest_message_reader = message_heap_.front();

    std::tuple<monotonic_clock::time_point, uint32_t> next_oldest_message_time =
        std::get<2>(next_oldest_message_reader)->oldest_message(channel_index_);

    if (std::get<0>(next_oldest_message_time) == std::get<0>(oldest_message) &&
        std::get<1>(next_oldest_message_time) == std::get<1>(oldest_message)) {
      // Pop the message reader pointer.
      std::pop_heap(message_heap_.begin(), message_heap_.end(),
                    &SplitMessageReaderHeapCompare);
      message_heap_.pop_back();

      // Pop the next oldest message.  This re-pushes any messages from the
      // reader.
      std::tuple<monotonic_clock::time_point, uint32_t,
                 FlatbufferVector<MessageHeader>>
          next_oldest_message = std::get<2>(next_oldest_message_reader)
                                    ->PopOldest(channel_index_);

      // And make sure the message matches in it's entirety.
      CHECK(std::get<2>(oldest_message).span() ==
            std::get<2>(next_oldest_message).span())
          << ": Data at the same timestamp doesn't match.";
    } else {
      break;
    }
  }

  return oldest_message;
}

std::tuple<monotonic_clock::time_point, uint32_t,
           FlatbufferVector<MessageHeader>>
TimestampMerger::PopTimestampHeap() {
  // Pop the oldest message reader pointer off the heap.
  CHECK_GT(timestamp_heap_.size(), 0u);

  std::tuple<monotonic_clock::time_point, uint32_t, SplitMessageReader *>
      oldest_timestamp_reader = timestamp_heap_.front();

  std::pop_heap(timestamp_heap_.begin(), timestamp_heap_.end(),
                &SplitMessageReaderHeapCompare);
  timestamp_heap_.pop_back();

  CHECK(node_index_ != -1) << ": Timestamps in a single node environment";

  // Pop the oldest message.  This re-pushes any timestamps from the reader to
  // the timestamp heap.
  std::tuple<monotonic_clock::time_point, uint32_t,
             FlatbufferVector<MessageHeader>>
      oldest_timestamp = std::get<2>(oldest_timestamp_reader)
                             ->PopOldest(channel_index_, node_index_);

  // Confirm that the time we have recorded matches.
  CHECK_EQ(std::get<0>(oldest_timestamp), std::get<0>(oldest_timestamp_reader));
  CHECK_EQ(std::get<1>(oldest_timestamp), std::get<1>(oldest_timestamp_reader));

  // TODO(austin): What if we get duplicate timestamps?

  return oldest_timestamp;
}

std::tuple<TimestampMerger::DeliveryTimestamp, FlatbufferVector<MessageHeader>>
TimestampMerger::PopOldest() {
  if (has_timestamps_) {
    CHECK_GT(message_heap_.size(), 0u)
        << ": Missing data from source node, no data available to match "
           "timestamp on "
        << configuration::CleanedChannelToString(
               configuration_->channels()->Get(channel_index_));

    std::tuple<monotonic_clock::time_point, uint32_t,
               FlatbufferVector<MessageHeader>>
        oldest_timestamp = PopTimestampHeap();

    TimestampMerger::DeliveryTimestamp timestamp;
    timestamp.monotonic_event_time =
        monotonic_clock::time_point(chrono::nanoseconds(
            std::get<2>(oldest_timestamp).message().monotonic_sent_time()));
    timestamp.realtime_event_time =
        realtime_clock::time_point(chrono::nanoseconds(
            std::get<2>(oldest_timestamp).message().realtime_sent_time()));

    // Consistency check.
    CHECK_EQ(timestamp.monotonic_event_time, std::get<0>(oldest_timestamp));
    CHECK_EQ(std::get<2>(oldest_timestamp).message().queue_index(),
             std::get<1>(oldest_timestamp));

    monotonic_clock::time_point remote_timestamp_monotonic_time(
        chrono::nanoseconds(
            std::get<2>(oldest_timestamp).message().monotonic_remote_time()));

    while (true) {
      // Ok, now try grabbing data until we find one which matches.
      std::tuple<monotonic_clock::time_point, uint32_t,
                 FlatbufferVector<MessageHeader>>
          oldest_message = PopMessageHeap();

      // Time at which the message was sent (this message is written from the
      // sending node's perspective.
      monotonic_clock::time_point remote_monotonic_time(chrono::nanoseconds(
          std::get<2>(oldest_message).message().monotonic_sent_time()));

      if (remote_monotonic_time < remote_timestamp_monotonic_time) {
        LOG(INFO) << "Undelivered message, skipping.  Remote time is "
                  << remote_monotonic_time << " timestamp is "
                  << remote_timestamp_monotonic_time << " on channel "
                  << channel_index_;
        continue;
      }

      timestamp.monotonic_remote_time = remote_monotonic_time;
      timestamp.realtime_remote_time =
          realtime_clock::time_point(chrono::nanoseconds(
              std::get<2>(oldest_message).message().realtime_sent_time()));
      timestamp.remote_queue_index =
          std::get<2>(oldest_message).message().queue_index();

      CHECK_EQ(remote_monotonic_time, remote_timestamp_monotonic_time);
      CHECK_EQ(timestamp.remote_queue_index, std::get<1>(oldest_timestamp));

      return std::make_tuple(timestamp, std::get<2>(oldest_message));
    }
  } else {
    std::tuple<monotonic_clock::time_point, uint32_t,
               FlatbufferVector<MessageHeader>>
        oldest_message = PopMessageHeap();

    TimestampMerger::DeliveryTimestamp timestamp;
    timestamp.monotonic_event_time =
        monotonic_clock::time_point(chrono::nanoseconds(
            std::get<2>(oldest_message).message().monotonic_sent_time()));
    timestamp.realtime_event_time =
        realtime_clock::time_point(chrono::nanoseconds(
            std::get<2>(oldest_message).message().realtime_sent_time()));
    timestamp.remote_queue_index = 0xffffffff;

    CHECK_EQ(std::get<0>(oldest_message), timestamp.monotonic_event_time);
    CHECK_EQ(std::get<1>(oldest_message),
             std::get<2>(oldest_message).message().queue_index());

    return std::make_tuple(timestamp, std::get<2>(oldest_message));
  }
}

namespace {
std::vector<std::unique_ptr<SplitMessageReader>> MakeSplitMessageReaders(
    const std::vector<std::vector<std::string>> &filenames) {
  CHECK_GT(filenames.size(), 0u);
  // Build up all the SplitMessageReaders.
  std::vector<std::unique_ptr<SplitMessageReader>> result;
  for (const std::vector<std::string> &filenames : filenames) {
    result.emplace_back(std::make_unique<SplitMessageReader>(filenames));
  }
  return result;
}
}  // namespace

ChannelMerger::ChannelMerger(
    const std::vector<std::vector<std::string>> &filenames)
    : split_message_readers_(MakeSplitMessageReaders(filenames)),
      log_file_header_(
          CopyFlatBuffer(split_message_readers_[0]->log_file_header())) {
  // Now, confirm that the configuration matches for each and pick a start time.
  // Also return the list of possible nodes.
  for (const std::unique_ptr<SplitMessageReader> &reader :
       split_message_readers_) {
    CHECK(CompareFlatBuffer(log_file_header_.message().configuration(),
                            reader->log_file_header()->configuration()))
        << ": Replaying log files with different configurations isn't "
           "supported";
  }

  nodes_ = configuration::GetNodes(configuration());
}

bool ChannelMerger::SetNode(const Node *target_node) {
  std::vector<SplitMessageReader *> split_message_readers;
  for (const std::unique_ptr<SplitMessageReader> &reader :
       split_message_readers_) {
    split_message_readers.emplace_back(reader.get());
  }

  // Go find a log_file_header for this node.
  {
    bool found_node = false;

    for (const std::unique_ptr<SplitMessageReader> &reader :
         split_message_readers_) {
      if (CompareFlatBuffer(reader->node(), target_node)) {
        if (!found_node) {
          found_node = true;
          log_file_header_ = CopyFlatBuffer(reader->log_file_header());
        } else {
          // And then make sure all the other files have matching headers.
          CHECK(
              CompareFlatBuffer(log_file_header(), reader->log_file_header()));
        }
      }
    }

    if (!found_node) {
      LOG(WARNING) << "Failed to find log file for node "
                   << FlatbufferToJson(target_node);
      return false;
    }
  }

  // Build up all the timestamp mergers.  This connects up all the
  // SplitMessageReaders.
  timestamp_mergers_.reserve(configuration()->channels()->size());
  for (size_t channel_index = 0;
       channel_index < configuration()->channels()->size(); ++channel_index) {
    timestamp_mergers_.emplace_back(
        configuration(), split_message_readers, channel_index,
        configuration::GetNode(configuration(), target_node), this);
  }

  // And prime everything.
  size_t split_message_reader_index = 0;
  for (std::unique_ptr<SplitMessageReader> &split_message_reader :
       split_message_readers_) {
    if (split_message_reader->QueueMessages(
            split_message_reader->monotonic_start_time())) {
      split_message_reader_heap_.push_back(std::make_pair(
          split_message_reader->queue_data_time(), split_message_reader_index));

      std::push_heap(split_message_reader_heap_.begin(),
                     split_message_reader_heap_.end(), ChannelHeapCompare);
    }
    ++split_message_reader_index;
  }

  node_ = configuration::GetNodeOrDie(configuration(), target_node);
  return true;
}

monotonic_clock::time_point ChannelMerger::OldestMessage() const {
  if (channel_heap_.size() == 0u) {
    return monotonic_clock::max_time;
  }
  return channel_heap_.front().first;
}

void ChannelMerger::PushChannelHeap(monotonic_clock::time_point timestamp,
                                    int channel_index) {
  // Pop and recreate the heap if it has already been pushed.  And since we are
  // pushing again, we don't need to clear pushed.
  if (timestamp_mergers_[channel_index].pushed()) {
    channel_heap_.erase(std::find_if(
        channel_heap_.begin(), channel_heap_.end(),
        [channel_index](const std::pair<monotonic_clock::time_point, int> x) {
          return x.second == channel_index;
        }));
    std::make_heap(channel_heap_.begin(), channel_heap_.end(),
                   ChannelHeapCompare);
  }

  channel_heap_.push_back(std::make_pair(timestamp, channel_index));

  // The default sort puts the newest message first.  Use a custom comparator to
  // put the oldest message first.
  std::push_heap(channel_heap_.begin(), channel_heap_.end(),
                 ChannelHeapCompare);
}

std::tuple<TimestampMerger::DeliveryTimestamp, int,
           FlatbufferVector<MessageHeader>>
ChannelMerger::PopOldest() {
  CHECK(channel_heap_.size() > 0);
  std::pair<monotonic_clock::time_point, int> oldest_channel_data =
      channel_heap_.front();
  int channel_index = oldest_channel_data.second;
  std::pop_heap(channel_heap_.begin(), channel_heap_.end(),
                &ChannelHeapCompare);
  channel_heap_.pop_back();
  timestamp_mergers_[channel_index].set_pushed(false);

  TimestampMerger *merger = &timestamp_mergers_[channel_index];

  // Merger auto-pushes from here, but doesn't fetch anything new from the log
  // file.
  std::tuple<TimestampMerger::DeliveryTimestamp,
             FlatbufferVector<MessageHeader>>
      message = merger->PopOldest();

  QueueMessages(OldestMessage());

  return std::make_tuple(std::get<0>(message), channel_index,
                         std::move(std::get<1>(message)));
}

void ChannelMerger::QueueMessages(
    monotonic_clock::time_point oldest_message_time) {
  // Pop and re-queue readers until they are all caught up.
  while (true) {
    if (split_message_reader_heap_.size() == 0) {
      return;
    }
    std::pair<monotonic_clock::time_point, int> oldest_channel_data =
        split_message_reader_heap_.front();

    // No work to do, bail.
    if (oldest_channel_data.first > oldest_message_time) {
      return;
    }

    // Drop it off the heap.
    std::pop_heap(split_message_reader_heap_.begin(),
                  split_message_reader_heap_.end(), &ChannelHeapCompare);
    split_message_reader_heap_.pop_back();

    // And if there is data left in the log file, push it back on the heap with
    // the updated time.
    const int split_message_reader_index = oldest_channel_data.second;
    if (split_message_readers_[split_message_reader_index]->QueueMessages(
            oldest_message_time)) {
      split_message_reader_heap_.push_back(std::make_pair(
          split_message_readers_[split_message_reader_index]->queue_data_time(),
          split_message_reader_index));

      std::push_heap(split_message_reader_heap_.begin(),
                     split_message_reader_heap_.end(), ChannelHeapCompare);
    }
  }
}

}  // namespace logger
}  // namespace aos
