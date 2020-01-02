#include "aos/events/logging/logfile_utils.h"

#include <fcntl.h>
#include <limits.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>

#include <vector>

#include "aos/configuration.h"
#include "aos/events/logging/logger_generated.h"
#include "flatbuffers/flatbuffers.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_int32(flush_size, 1000000,
             "Number of outstanding bytes to allow before flushing to disk.");

namespace aos {
namespace logger {

namespace chrono = std::chrono;

DetachedBufferWriter::DetachedBufferWriter(std::string_view filename)
    : fd_(open(std::string(filename).c_str(),
               O_RDWR | O_CLOEXEC | O_CREAT | O_EXCL, 0774)) {
  PCHECK(fd_ != -1) << ": Failed to open " << filename;
}

DetachedBufferWriter::~DetachedBufferWriter() {
  Flush();
  PLOG_IF(ERROR, close(fd_) == -1) << " Failed to close logfile";
}

void DetachedBufferWriter::QueueSizedFlatbuffer(
    flatbuffers::FlatBufferBuilder *fbb) {
  QueueSizedFlatbuffer(fbb->Release());
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
      data_offset =
          fbb->CreateVector(static_cast<uint8_t *>(context.data), context.size);
      break;

    case LogType::kLogDeliveryTimeOnly:
      break;
  }

  MessageHeader::Builder message_header_builder(*fbb);
  message_header_builder.add_channel_index(channel_index);
  message_header_builder.add_queue_index(context.queue_index);
  message_header_builder.add_monotonic_sent_time(
      context.monotonic_event_time.time_since_epoch().count());
  message_header_builder.add_realtime_sent_time(
      context.realtime_event_time.time_since_epoch().count());

  switch (log_type) {
    case LogType::kLogMessage:
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
    : fd_(open(std::string(filename).c_str(), O_RDONLY | O_CLOEXEC)) {
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
  return result;
}

SortedMessageReader::SortedMessageReader(std::string_view filename)
    : message_reader_(filename) {
  channels_.resize(configuration()->channels()->size());

  QueueMessages();
}

void SortedMessageReader::EmplaceDataBack(
    FlatbufferVector<MessageHeader> &&new_data) {
  const monotonic_clock::time_point timestamp = monotonic_clock::time_point(
      chrono::nanoseconds(new_data.message().monotonic_sent_time()));
  const size_t channel_index = new_data.message().channel_index();
  CHECK_LT(channel_index, channels_.size());

  if (channels_[channel_index].data.size() == 0) {
    channels_[channel_index].oldest_timestamp = timestamp;
    PushChannelHeap(timestamp, channel_index);
  }
  channels_[channel_index].data.emplace_back(std::move(new_data));
}

namespace {

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

void SortedMessageReader::PushChannelHeap(monotonic_clock::time_point timestamp,
                                          int channel_index) {
  channel_heap_.push_back(std::make_pair(timestamp, channel_index));

  // The default sort puts the newest message first.  Use a custom comparator to
  // put the oldest message first.
  std::push_heap(channel_heap_.begin(), channel_heap_.end(),
                 ChannelHeapCompare);
}

void SortedMessageReader::QueueMessages() {
  while (true) {
    // Don't queue if we have enough data already.
    // When a log file starts, there should be a message from each channel.
    // Those messages might be very old. Make sure to read a chunk past the
    // starting time.
    if (channel_heap_.size() > 0 &&
        message_reader_.newest_timestamp() >
            std::max(oldest_message().first, monotonic_start_time()) +
                message_reader_.max_out_of_order_duration()) {
      break;
    }

    if (std::optional<FlatbufferVector<MessageHeader>> msg =
            message_reader_.ReadMessage()) {
      EmplaceDataBack(std::move(msg.value()));
    } else {
      break;
    }
  }
}

std::tuple<monotonic_clock::time_point, int, FlatbufferVector<MessageHeader>>
SortedMessageReader::PopOldestChannel() {
  std::pair<monotonic_clock::time_point, int> oldest_channel_data =
      channel_heap_.front();
  std::pop_heap(channel_heap_.begin(), channel_heap_.end(),
                &ChannelHeapCompare);
  channel_heap_.pop_back();

  struct ChannelData &channel = channels_[oldest_channel_data.second];

  FlatbufferVector<MessageHeader> front = std::move(channel.front());

  channel.data.pop_front();

  // Re-push it and update the oldest timestamp.
  if (channel.data.size() != 0) {
    const monotonic_clock::time_point timestamp = monotonic_clock::time_point(
        chrono::nanoseconds(channel.front().message().monotonic_sent_time()));
    PushChannelHeap(timestamp, oldest_channel_data.second);
    channel.oldest_timestamp = timestamp;
  } else {
    channel.oldest_timestamp = monotonic_clock::min_time;
  }

  if (oldest_channel_data.first > message_reader_.queue_data_time()) {
    QueueMessages();
  }

  return std::make_tuple(oldest_channel_data.first, oldest_channel_data.second,
                         std::move(front));
}

}  // namespace logger
}  // namespace aos
