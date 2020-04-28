#include "aos/events/logging/logfile_utils.h"

#include <fcntl.h>
#include <limits.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>

#include <vector>

#include "absl/strings/escaping.h"
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
  written_size_ += written;
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
  written_size_ += written;

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
  if (data_size == sizeof(flatbuffers::uoffset_t)) {
    LOG(ERROR) << "Size of data is zero.  Log file end is corrupted, skipping.";
    LOG(ERROR) << "  Rest of log file is "
               << absl::BytesToHexString(std::string_view(
                      reinterpret_cast<const char *>(data_.data() +
                                                     consumed_data_),
                      data_.size() - consumed_data_));
    return absl::Span<const uint8_t>();
  }
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

  max_out_of_order_duration_ =
      std::chrono::nanoseconds(log_file_header()->max_out_of_order_duration());

  VLOG(1) << "Opened " << filename << " as node "
          << FlatbufferToJson(log_file_header()->node());
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
  VLOG(2) << "Read from " << filename() << " data " << FlatbufferToJson(result);
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
    monotonic_clock::time_point last_dequeued_time) {
  // TODO(austin): Once we are happy that everything works, read a 256kb chunk
  // to reduce the need to re-heap down below.

  // Special case no more data.  Otherwise we blow up on the CHECK statement
  // confirming that we have enough data queued.
  if (at_end_) {
    return false;
  }

  // If this isn't the first time around, confirm that we had enough data queued
  // to follow the contract.
  if (time_to_queue_ != monotonic_clock::min_time) {
    CHECK_LE(last_dequeued_time,
             newest_timestamp() - max_out_of_order_duration())
        << " node " << FlatbufferToJson(node()) << " on " << this;

    // Bail if there is enough data already queued.
    if (last_dequeued_time < time_to_queue_) {
      VLOG(1) << "All up to date on " << this << ", dequeued "
              << last_dequeued_time << " queue time " << time_to_queue_;
      return true;
    }
  } else {
    // Startup takes a special dance.  We want to queue up until the start time,
    // but we then want to find the next message to read.  The conservative
    // answer is to immediately trigger a second requeue to get things moving.
    time_to_queue_ = monotonic_start_time();
    QueueMessages(time_to_queue_);
  }

  // If we are asked to queue, queue for at least max_out_of_order_duration past
  // the last known time in the log file (ie the newest timestep read).  As long
  // as we requeue exactly when time_to_queue_ is dequeued and go no further, we
  // are safe.  And since we pop in order, that works.
  //
  // Special case the start of the log file.  There should be at most 1 message
  // from each channel at the start of the log file.  So always force the start
  // of the log file to just be read.
  time_to_queue_ = std::max(time_to_queue_, newest_timestamp());
  VLOG(1) << "Queueing, going until " << time_to_queue_ << " " << filename();

  bool was_emplaced = false;
  while (true) {
    // Stop if we have enough.
    if (newest_timestamp() > time_to_queue_ + max_out_of_order_duration() &&
        was_emplaced) {
      VLOG(1) << "Done queueing on " << this << ", queued to "
              << newest_timestamp() << " with requeue time " << time_to_queue_;
      return true;
    }

    if (std::optional<FlatbufferVector<MessageHeader>> msg =
            message_reader_->ReadMessage()) {
      const MessageHeader &header = msg.value().message();

      const monotonic_clock::time_point timestamp = monotonic_clock::time_point(
          chrono::nanoseconds(header.monotonic_sent_time()));

      if (VLOG_IS_ON(2)) {
        LOG(INFO) << "Queued " << this << " " << filename()
                << " ttq: " << time_to_queue_ << " now " << newest_timestamp()
                << " start time " << monotonic_start_time() << " "
                << FlatbufferToJson(&header);
      } else if (VLOG_IS_ON(1)) {
        FlatbufferVector<MessageHeader> copy = msg.value();
        copy.mutable_message()->clear_data();
        LOG(INFO) << "Queued " << this << " " << filename()
                << " ttq: " << time_to_queue_ << " now " << newest_timestamp()
                << " start time " << monotonic_start_time() << " "
                << FlatbufferToJson(copy);
      }

      const int channel_index = header.channel_index();
      was_emplaced = channels_to_write_[channel_index]->emplace_back(
          std::move(msg.value()));
      if (was_emplaced) {
        newest_timestamp_ = std::max(newest_timestamp_, timestamp);
      }
    } else {
      if (!NextLogFile()) {
        VLOG(1) << "End of log file " << filenames_.back();
        at_end_ = true;
        for (MessageHeaderQueue *queue : channels_to_write_) {
          if (queue == nullptr || queue->timestamp_merger == nullptr) {
            continue;
          }
          queue->timestamp_merger->NoticeAtEnd();
        }
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

  VLOG(1) << "  Configuring merger " << this << " for channel " << channel_index
          << " "
          << configuration::CleanedChannelToString(
                 configuration()->channels()->Get(channel_index));

  MessageHeaderQueue *message_header_queue = nullptr;

  // Figure out if this log file is from our point of view, or the other node's
  // point of view.
  if (node() == reinterpreted_target_node) {
    VLOG(1) << "    Replaying as logged node " << filename();

    if (configuration::ChannelIsSendableOnNode(channel, node())) {
      VLOG(1) << "      Data on node";
      message_header_queue = &(channels_[channel_index].data);
    } else if (configuration::ChannelIsReadableOnNode(channel, node())) {
      VLOG(1) << "      Timestamps on node";
      message_header_queue =
          &(channels_[channel_index].timestamps[configuration::GetNodeIndex(
              configuration(), node())]);
    } else {
      VLOG(1) << "     Dropping";
    }
  } else {
    VLOG(1) << "    Replaying as other node " << filename();
    // We are replaying from another node's point of view.  The only interesting
    // data is data that is sent from our node and received on theirs.
    if (configuration::ChannelIsReadableOnNode(channel,
                                               reinterpreted_target_node) &&
        configuration::ChannelIsSendableOnNode(channel, node())) {
      VLOG(1) << "      Readable on target node";
      // Data from another node.
      message_header_queue = &(channels_[channel_index].data);
    } else {
      VLOG(1) << "      Dropping";
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
  const std::tuple<monotonic_clock::time_point, uint32_t, const MessageHeader *>
      timestamp = channels_[channel_index].data.front_timestamp();
  FlatbufferVector<MessageHeader> front =
      std::move(channels_[channel_index].data.front());
  channels_[channel_index].data.pop_front();

  VLOG(1) << "Popped " << this << " " << std::get<0>(timestamp);

  QueueMessages(std::get<0>(timestamp));

  return std::make_tuple(std::get<0>(timestamp), std::get<1>(timestamp),
                         std::move(front));
}

std::tuple<monotonic_clock::time_point, uint32_t,
           FlatbufferVector<MessageHeader>>
SplitMessageReader::PopOldest(int channel, int node_index) {
  CHECK_GT(channels_[channel].timestamps[node_index].size(), 0u);
  const std::tuple<monotonic_clock::time_point, uint32_t, const MessageHeader *>
      timestamp = channels_[channel].timestamps[node_index].front_timestamp();
  FlatbufferVector<MessageHeader> front =
      std::move(channels_[channel].timestamps[node_index].front());
  channels_[channel].timestamps[node_index].pop_front();

  VLOG(1) << "Popped " << this << " " << std::get<0>(timestamp);

  QueueMessages(std::get<0>(timestamp));

  return std::make_tuple(std::get<0>(timestamp), std::get<1>(timestamp),
                         std::move(front));
}

bool SplitMessageReader::MessageHeaderQueue::emplace_back(
    FlatbufferVector<MessageHeader> &&msg) {
  CHECK(split_reader != nullptr);

  // If there is no timestamp merger for this queue, nobody is listening.  Drop
  // the message.  This happens when a log file from another node is replayed,
  // and the timestamp mergers down stream just don't care.
  if (timestamp_merger == nullptr) {
    return false;
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

  return true;
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
  VLOG(1) << "Configuring channel " << channel_index << " target node "
          << FlatbufferToJson(target_node);
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
    std::tuple<monotonic_clock::time_point, uint32_t, const MessageHeader *>
        timestamp,
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

std::tuple<monotonic_clock::time_point, uint32_t, const MessageHeader *>
TimestampMerger::oldest_message() const {
  CHECK_GT(message_heap_.size(), 0u);
  std::tuple<monotonic_clock::time_point, uint32_t, SplitMessageReader *>
      oldest_message_reader = message_heap_.front();
  return std::get<2>(oldest_message_reader)->oldest_message(channel_index_);
}

std::tuple<monotonic_clock::time_point, uint32_t, const MessageHeader *>
TimestampMerger::oldest_timestamp() const {
  CHECK_GT(timestamp_heap_.size(), 0u);
  std::tuple<monotonic_clock::time_point, uint32_t, SplitMessageReader *>
      oldest_message_reader = timestamp_heap_.front();
  return std::get<2>(oldest_message_reader)
      ->oldest_message(channel_index_, node_index_);
}

void TimestampMerger::PushTimestampHeap(
    std::tuple<monotonic_clock::time_point, uint32_t, const MessageHeader *>
        timestamp,
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

    std::tuple<monotonic_clock::time_point, uint32_t, const MessageHeader *>
        next_oldest_message_time = std::get<2>(next_oldest_message_reader)
                                       ->oldest_message(channel_index_);

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

TimestampMerger::DeliveryTimestamp TimestampMerger::OldestTimestamp() const {
  if (!has_timestamps_ || timestamp_heap_.size() == 0u) {
    return TimestampMerger::DeliveryTimestamp{};
  }

  std::tuple<monotonic_clock::time_point, uint32_t, SplitMessageReader *>
      oldest_timestamp_reader = timestamp_heap_.front();

  std::tuple<monotonic_clock::time_point, uint32_t, const MessageHeader *>
      oldest_timestamp = std::get<2>(oldest_timestamp_reader)
                             ->oldest_message(channel_index_, node_index_);

  TimestampMerger::DeliveryTimestamp timestamp;
  timestamp.monotonic_event_time =
      monotonic_clock::time_point(chrono::nanoseconds(
          std::get<2>(oldest_timestamp)->monotonic_sent_time()));
  timestamp.realtime_event_time = realtime_clock::time_point(
      chrono::nanoseconds(std::get<2>(oldest_timestamp)->realtime_sent_time()));

  timestamp.monotonic_remote_time =
      monotonic_clock::time_point(chrono::nanoseconds(
          std::get<2>(oldest_timestamp)->monotonic_remote_time()));
  timestamp.realtime_remote_time =
      realtime_clock::time_point(chrono::nanoseconds(
          std::get<2>(oldest_timestamp)->realtime_remote_time()));

  timestamp.remote_queue_index = std::get<2>(oldest_timestamp)->queue_index();
  return timestamp;
}

std::tuple<TimestampMerger::DeliveryTimestamp, FlatbufferVector<MessageHeader>>
TimestampMerger::PopOldest() {
  if (has_timestamps_) {
    // Read the timestamps.
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

    // See if we have any data.  If not, pass the problem up the chain.
    if (message_heap_.size() == 0u) {
      VLOG(1) << "No data to match timestamp on "
              << configuration::CleanedChannelToString(
                     configuration_->channels()->Get(channel_index_));
      return std::make_tuple(timestamp,
                             std::move(std::get<2>(oldest_timestamp)));
    }

    while (true) {
      {
        // Ok, now try grabbing data until we find one which matches.
        std::tuple<monotonic_clock::time_point, uint32_t, const MessageHeader *>
            oldest_message_ref = oldest_message();

        // Time at which the message was sent (this message is written from the
        // sending node's perspective.
        monotonic_clock::time_point remote_monotonic_time(chrono::nanoseconds(
            std::get<2>(oldest_message_ref)->monotonic_sent_time()));

        if (remote_monotonic_time < remote_timestamp_monotonic_time) {
          VLOG(1) << "Undelivered message, skipping.  Remote time is "
                  << remote_monotonic_time << " timestamp is "
                  << remote_timestamp_monotonic_time << " on channel "
                  << channel_index_;
          PopMessageHeap();
          continue;
        } else if (remote_monotonic_time > remote_timestamp_monotonic_time) {
          VLOG(1) << "Data not found.  Remote time should be "
                  << remote_timestamp_monotonic_time << " on channel "
                  << channel_index_;
          return std::make_tuple(timestamp,
                                 std::move(std::get<2>(oldest_timestamp)));
        }

        timestamp.monotonic_remote_time = remote_monotonic_time;
      }

      std::tuple<monotonic_clock::time_point, uint32_t,
                 FlatbufferVector<MessageHeader>>
          oldest_message = PopMessageHeap();

      timestamp.realtime_remote_time =
          realtime_clock::time_point(chrono::nanoseconds(
              std::get<2>(oldest_message).message().realtime_sent_time()));
      timestamp.remote_queue_index =
          std::get<2>(oldest_message).message().queue_index();

      CHECK_EQ(timestamp.monotonic_remote_time,
               remote_timestamp_monotonic_time);

      CHECK_EQ(timestamp.remote_queue_index,
               std::get<2>(oldest_timestamp).message().remote_queue_index())
          << ": " << FlatbufferToJson(&std::get<2>(oldest_timestamp).message())
          << " data "
          << FlatbufferToJson(&std::get<2>(oldest_message).message());

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

void TimestampMerger::NoticeAtEnd() { channel_merger_->NoticeAtEnd(); }

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
          VLOG(1) << "Found log file " << reader->filename() << " with node "
                  << FlatbufferToJson(reader->node()) << " start_time "
                  << monotonic_start_time();
        } else {
          // And then make sure all the other files have matching headers.
          CHECK(CompareFlatBuffer(log_file_header(), reader->log_file_header()))
              << ": " << FlatbufferToJson(log_file_header()) << " reader "
              << FlatbufferToJson(reader->log_file_header());
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
  for (std::unique_ptr<SplitMessageReader> &split_message_reader :
       split_message_readers_) {
    split_message_reader->QueueMessages(
        split_message_reader->monotonic_start_time());
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

TimestampMerger::DeliveryTimestamp ChannelMerger::OldestTimestamp() const {
  if (timestamp_heap_.size() == 0u) {
    return TimestampMerger::DeliveryTimestamp{};
  }
  return timestamp_mergers_[timestamp_heap_.front().second].OldestTimestamp();
}

TimestampMerger::DeliveryTimestamp ChannelMerger::OldestTimestampForChannel(
    int channel) const {
  // If we didn't find any data for this node, we won't have any mergers. Return
  // an invalid timestamp in that case.
  if (timestamp_mergers_.size() <= static_cast<size_t>(channel)) {
    TimestampMerger::DeliveryTimestamp result;
    return result;
  }
  return timestamp_mergers_[channel].OldestTimestamp();
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

    if (timestamp_mergers_[channel_index].has_timestamps()) {
      timestamp_heap_.erase(std::find_if(
          timestamp_heap_.begin(), timestamp_heap_.end(),
          [channel_index](const std::pair<monotonic_clock::time_point, int> x) {
            return x.second == channel_index;
          }));
      std::make_heap(timestamp_heap_.begin(), timestamp_heap_.end(),
                     ChannelHeapCompare);
    }
  }

  channel_heap_.push_back(std::make_pair(timestamp, channel_index));

  // The default sort puts the newest message first.  Use a custom comparator to
  // put the oldest message first.
  std::push_heap(channel_heap_.begin(), channel_heap_.end(),
                 ChannelHeapCompare);

  if (timestamp_mergers_[channel_index].has_timestamps()) {
    timestamp_heap_.push_back(std::make_pair(timestamp, channel_index));
    std::push_heap(timestamp_heap_.begin(), timestamp_heap_.end(),
                   ChannelHeapCompare);
  }
}

std::tuple<TimestampMerger::DeliveryTimestamp, int,
           FlatbufferVector<MessageHeader>>
ChannelMerger::PopOldest() {
  CHECK_GT(channel_heap_.size(), 0u);
  std::pair<monotonic_clock::time_point, int> oldest_channel_data =
      channel_heap_.front();
  int channel_index = oldest_channel_data.second;
  std::pop_heap(channel_heap_.begin(), channel_heap_.end(),
                &ChannelHeapCompare);
  channel_heap_.pop_back();

  timestamp_mergers_[channel_index].set_pushed(false);

  TimestampMerger *merger = &timestamp_mergers_[channel_index];

  if (merger->has_timestamps()) {
    CHECK_GT(timestamp_heap_.size(), 0u);
    std::pair<monotonic_clock::time_point, int> oldest_timestamp_data =
        timestamp_heap_.front();
    CHECK(oldest_timestamp_data == oldest_channel_data)
        << ": Timestamp heap out of sync.";
    std::pop_heap(timestamp_heap_.begin(), timestamp_heap_.end(),
                  &ChannelHeapCompare);
    timestamp_heap_.pop_back();
  }

  // Merger handles any queueing needed from here.
  std::tuple<TimestampMerger::DeliveryTimestamp,
             FlatbufferVector<MessageHeader>>
      message = merger->PopOldest();

  return std::make_tuple(std::get<0>(message), channel_index,
                         std::move(std::get<1>(message)));
}

std::string SplitMessageReader::MessageHeaderQueue::DebugString() const {
  std::stringstream ss;
  for (size_t i = 0; i < data_.size(); ++i) {
    if (timestamps) {
      ss << "        msg: ";
    } else {
      ss << "        timestamp: ";
    }
    ss << monotonic_clock::time_point(std::chrono::nanoseconds(
              data_[i].message().monotonic_sent_time()))
       << " ("
       << realtime_clock::time_point(
              std::chrono::nanoseconds(data_[i].message().realtime_sent_time()))
       << ") " << data_[i].message().queue_index();
    if (timestamps) {
      ss << "  <- remote "
         << monotonic_clock::time_point(std::chrono::nanoseconds(
                data_[i].message().monotonic_remote_time()))
         << " ("
         << realtime_clock::time_point(std::chrono::nanoseconds(
                data_[i].message().realtime_remote_time()))
         << ")";
    }
    ss << "\n";
  }

  return ss.str();
}

std::string SplitMessageReader::DebugString(int channel) const {
  std::stringstream ss;
  ss << "[\n";
  ss << channels_[channel].data.DebugString();
  ss << "      ]";
  return ss.str();
}

std::string SplitMessageReader::DebugString(int channel, int node_index) const {
  std::stringstream ss;
  ss << "[\n";
  ss << channels_[channel].timestamps[node_index].DebugString();
  ss << "      ]";
  return ss.str();
}

std::string TimestampMerger::DebugString() const {
  std::stringstream ss;

  if (timestamp_heap_.size() > 0) {
    ss << "    timestamp_heap {\n";
    std::vector<
        std::tuple<monotonic_clock::time_point, uint32_t, SplitMessageReader *>>
        timestamp_heap = timestamp_heap_;
    while (timestamp_heap.size() > 0u) {
      std::tuple<monotonic_clock::time_point, uint32_t, SplitMessageReader *>
          oldest_timestamp_reader = timestamp_heap.front();

      ss << "      " << std::get<2>(oldest_timestamp_reader) << " "
         << std::get<0>(oldest_timestamp_reader) << " queue_index ("
         << std::get<1>(oldest_timestamp_reader) << ") ttq "
         << std::get<2>(oldest_timestamp_reader)->time_to_queue() << " "
         << std::get<2>(oldest_timestamp_reader)->filename() << " -> "
         << std::get<2>(oldest_timestamp_reader)
                ->DebugString(channel_index_, node_index_)
         << "\n";

      std::pop_heap(timestamp_heap.begin(), timestamp_heap.end(),
                    &SplitMessageReaderHeapCompare);
      timestamp_heap.pop_back();
    }
    ss << "    }\n";
  }

  ss << "    message_heap {\n";
  {
    std::vector<
        std::tuple<monotonic_clock::time_point, uint32_t, SplitMessageReader *>>
        message_heap = message_heap_;
    while (message_heap.size() > 0u) {
      std::tuple<monotonic_clock::time_point, uint32_t, SplitMessageReader *>
          oldest_message_reader = message_heap.front();

      ss << "      " << std::get<2>(oldest_message_reader) << " "
         << std::get<0>(oldest_message_reader) << " queue_index ("
         << std::get<1>(oldest_message_reader) << ") ttq "
         << std::get<2>(oldest_message_reader)->time_to_queue() << " "
         << std::get<2>(oldest_message_reader)->filename() << " -> "
         << std::get<2>(oldest_message_reader)->DebugString(channel_index_)
         << "\n";

      std::pop_heap(message_heap.begin(), message_heap.end(),
                    &SplitMessageReaderHeapCompare);
      message_heap.pop_back();
    }
  }
  ss << "    }";

  return ss.str();
}

std::string ChannelMerger::DebugString() const {
  std::stringstream ss;
  ss << "start_time " << realtime_start_time() << " " << monotonic_start_time()
     << "\n";
  ss << "channel_heap {\n";
  std::vector<std::pair<monotonic_clock::time_point, int>> channel_heap =
      channel_heap_;
  while (channel_heap.size() > 0u) {
    std::tuple<monotonic_clock::time_point, int> channel = channel_heap.front();
    ss << "  " << std::get<0>(channel) << " (" << std::get<1>(channel) << ") "
       << configuration::CleanedChannelToString(
              configuration()->channels()->Get(std::get<1>(channel)))
       << "\n";

    ss << timestamp_mergers_[std::get<1>(channel)].DebugString() << "\n";

    std::pop_heap(channel_heap.begin(), channel_heap.end(),
                  &ChannelHeapCompare);
    channel_heap.pop_back();
  }
  ss << "}";

  return ss.str();
}

}  // namespace logger
}  // namespace aos
