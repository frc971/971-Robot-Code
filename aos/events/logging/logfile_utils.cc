#include "aos/events/logging/logfile_utils.h"

#include <fcntl.h>
#include <limits.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>

#include <vector>

#include "aos/events/logging/logger_generated.h"
#include "flatbuffers/flatbuffers.h"

DEFINE_int32(flush_size, 1000000,
             "Number of outstanding bytes to allow before flushing to disk.");

namespace aos {
namespace logger {

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

}  // namespace logger
}  // namespace aos
