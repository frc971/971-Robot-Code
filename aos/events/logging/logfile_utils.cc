#include "aos/events/logging/logfile_utils.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>

#include <algorithm>
#include <climits>

#include "absl/strings/escaping.h"
#include "aos/configuration.h"
#include "aos/events/logging/snappy_encoder.h"
#include "aos/flatbuffer_merge.h"
#include "aos/util/file.h"
#include "flatbuffers/flatbuffers.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#if defined(__x86_64__)
#define ENABLE_LZMA (!__has_feature(memory_sanitizer))
#elif defined(__aarch64__)
#define ENABLE_LZMA (!__has_feature(memory_sanitizer))
#else
#define ENABLE_LZMA 0
#endif

#if ENABLE_LZMA
#include "aos/events/logging/lzma_encoder.h"
#endif

DEFINE_int32(flush_size, 128000,
             "Number of outstanding bytes to allow before flushing to disk.");
DEFINE_double(
    flush_period, 5.0,
    "Max time to let data sit in the queue before flushing in seconds.");

DEFINE_double(
    max_network_delay, 1.0,
    "Max time to assume a message takes to cross the network before we are "
    "willing to drop it from our buffers and assume it didn't make it.  "
    "Increasing this number can increase memory usage depending on the packet "
    "loss of your network or if the timestamps aren't logged for a message.");

DEFINE_double(
    max_out_of_order, -1,
    "If set, this overrides the max out of order duration for a log file.");

DEFINE_bool(workaround_double_headers, true,
            "Some old log files have two headers at the beginning.  Use the "
            "last header as the actual header.");

DEFINE_bool(crash_on_corrupt_message, true,
            "When true, MessageReader will crash the first time a message "
            "with corrupted format is found. When false, the crash will be "
            "suppressed, and any remaining readable messages will be "
            "evaluated to present verified vs corrupted stats.");

DEFINE_bool(ignore_corrupt_messages, false,
            "When true, and crash_on_corrupt_message is false, then any "
            "corrupt message found by MessageReader be silently ignored, "
            "providing access to all uncorrupted messages in a logfile.");

namespace aos::logger {
namespace {

namespace chrono = std::chrono;

template <typename T>
void PrintOptionalOrNull(std::ostream *os, const std::optional<T> &t) {
  if (t.has_value()) {
    *os << *t;
  } else {
    *os << "null";
  }
}
}  // namespace

DetachedBufferWriter::DetachedBufferWriter(
    std::string_view filename, std::unique_ptr<DetachedBufferEncoder> encoder)
    : filename_(filename), encoder_(std::move(encoder)) {
  if (!util::MkdirPIfSpace(filename, 0777)) {
    ran_out_of_space_ = true;
  } else {
    fd_ = open(std::string(filename).c_str(),
               O_RDWR | O_CLOEXEC | O_CREAT | O_EXCL, 0774);
    if (fd_ == -1 && errno == ENOSPC) {
      ran_out_of_space_ = true;
    } else {
      PCHECK(fd_ != -1) << ": Failed to open " << this->filename()
                        << " for writing";
      VLOG(1) << "Opened " << this->filename() << " for writing";
    }
  }
}

DetachedBufferWriter::~DetachedBufferWriter() {
  Close();
  if (ran_out_of_space_) {
    CHECK(acknowledge_ran_out_of_space_)
        << ": Unacknowledged out of disk space, log file was not completed";
  }
}

DetachedBufferWriter::DetachedBufferWriter(DetachedBufferWriter &&other) {
  *this = std::move(other);
}

// When other is destroyed "soon" (which it should be because we're getting an
// rvalue reference to it), it will flush etc all the data we have queued up
// (because that data will then be its data).
DetachedBufferWriter &DetachedBufferWriter::operator=(
    DetachedBufferWriter &&other) {
  std::swap(filename_, other.filename_);
  std::swap(encoder_, other.encoder_);
  std::swap(fd_, other.fd_);
  std::swap(ran_out_of_space_, other.ran_out_of_space_);
  std::swap(acknowledge_ran_out_of_space_, other.acknowledge_ran_out_of_space_);
  std::swap(iovec_, other.iovec_);
  std::swap(max_write_time_, other.max_write_time_);
  std::swap(max_write_time_bytes_, other.max_write_time_bytes_);
  std::swap(max_write_time_messages_, other.max_write_time_messages_);
  std::swap(total_write_time_, other.total_write_time_);
  std::swap(total_write_count_, other.total_write_count_);
  std::swap(total_write_messages_, other.total_write_messages_);
  std::swap(total_write_bytes_, other.total_write_bytes_);
  return *this;
}

void DetachedBufferWriter::QueueSpan(absl::Span<const uint8_t> span) {
  if (ran_out_of_space_) {
    // We don't want any later data to be written after space becomes
    // available, so refuse to write anything more once we've dropped data
    // because we ran out of space.
    VLOG(1) << "Ignoring span: " << span.size();
    return;
  }

  aos::monotonic_clock::time_point now;
  if (encoder_->may_bypass() && span.size() > 4096u) {
    // Over this threshold, we'll assume it's cheaper to add an extra
    // syscall to write the data immediately instead of copying it to
    // enqueue.

    // First, flush everything.
    while (encoder_->queue_size() > 0u) {
      Flush();
    }

    // Then, write it directly.
    const auto start = aos::monotonic_clock::now();
    const ssize_t written = write(fd_, span.data(), span.size());
    const auto end = aos::monotonic_clock::now();
    HandleWriteReturn(written, span.size());
    UpdateStatsForWrite(end - start, written, 1);
    now = end;
  } else {
    encoder_->Encode(CopySpanAsDetachedBuffer(span));
    now = aos::monotonic_clock::now();
  }

  FlushAtThreshold(now);
}

void DetachedBufferWriter::Close() {
  if (fd_ == -1) {
    return;
  }
  encoder_->Finish();
  while (encoder_->queue_size() > 0) {
    Flush();
  }
  if (close(fd_) == -1) {
    if (errno == ENOSPC) {
      ran_out_of_space_ = true;
    } else {
      PLOG(ERROR) << "Closing log file failed";
    }
  }
  fd_ = -1;
  VLOG(1) << "Closed " << filename();
}

void DetachedBufferWriter::Flush() {
  if (ran_out_of_space_) {
    // We don't want any later data to be written after space becomes available,
    // so refuse to write anything more once we've dropped data because we ran
    // out of space.
    if (encoder_) {
      VLOG(1) << "Ignoring queue: " << encoder_->queue().size();
      encoder_->Clear(encoder_->queue().size());
    } else {
      VLOG(1) << "No queue to ignore";
    }
    return;
  }

  const auto queue = encoder_->queue();
  if (queue.empty()) {
    return;
  }

  iovec_.clear();
  const size_t iovec_size = std::min<size_t>(queue.size(), IOV_MAX);
  iovec_.resize(iovec_size);
  size_t counted_size = 0;
  for (size_t i = 0; i < iovec_size; ++i) {
    iovec_[i].iov_base = const_cast<uint8_t *>(queue[i].data());
    iovec_[i].iov_len = queue[i].size();
    counted_size += iovec_[i].iov_len;
  }

  const auto start = aos::monotonic_clock::now();
  const ssize_t written = writev(fd_, iovec_.data(), iovec_.size());
  const auto end = aos::monotonic_clock::now();
  HandleWriteReturn(written, counted_size);

  encoder_->Clear(iovec_size);

  UpdateStatsForWrite(end - start, written, iovec_size);
}

void DetachedBufferWriter::HandleWriteReturn(ssize_t write_return,
                                             size_t write_size) {
  if (write_return == -1 && errno == ENOSPC) {
    ran_out_of_space_ = true;
    return;
  }
  PCHECK(write_return >= 0) << ": write failed";
  if (write_return < static_cast<ssize_t>(write_size)) {
    // Sometimes this happens instead of ENOSPC. On a real filesystem, this
    // never seems to happen in any other case. If we ever want to log to a
    // socket, this will happen more often. However, until we get there, we'll
    // just assume it means we ran out of space.
    ran_out_of_space_ = true;
    return;
  }
}

void DetachedBufferWriter::UpdateStatsForWrite(
    aos::monotonic_clock::duration duration, ssize_t written, int iovec_size) {
  if (duration > max_write_time_) {
    max_write_time_ = duration;
    max_write_time_bytes_ = written;
    max_write_time_messages_ = iovec_size;
  }
  total_write_time_ += duration;
  ++total_write_count_;
  total_write_messages_ += iovec_size;
  total_write_bytes_ += written;
}

void DetachedBufferWriter::FlushAtThreshold(
    aos::monotonic_clock::time_point now) {
  if (ran_out_of_space_) {
    // We don't want any later data to be written after space becomes available,
    // so refuse to write anything more once we've dropped data because we ran
    // out of space.
    if (encoder_) {
      VLOG(1) << "Ignoring queue: " << encoder_->queue().size();
      encoder_->Clear(encoder_->queue().size());
    } else {
      VLOG(1) << "No queue to ignore";
    }
    return;
  }

  // We don't want to flush the first time through.  Otherwise we will flush as
  // the log file header might be compressing, defeating any parallelism and
  // queueing there.
  if (last_flush_time_ == aos::monotonic_clock::min_time) {
    last_flush_time_ = now;
  }

  // Flush if we are at the max number of iovs per writev, because there's no
  // point queueing up any more data in memory. Also flush once we have enough
  // data queued up or if it has been long enough.
  while (encoder_->queued_bytes() > static_cast<size_t>(FLAGS_flush_size) ||
         encoder_->queue_size() >= IOV_MAX ||
         now > last_flush_time_ +
                   chrono::duration_cast<chrono::nanoseconds>(
                       chrono::duration<double>(FLAGS_flush_period))) {
    last_flush_time_ = now;
    Flush();
  }
}

// Do the magic dance to convert the endianness of the data and append it to the
// buffer.
namespace {

// TODO(austin): Look at the generated code to see if building the header is
// efficient or not.
template <typename T>
uint8_t *Push(uint8_t *buffer, const T data) {
  const T endian_data = flatbuffers::EndianScalar<T>(data);
  std::memcpy(buffer, &endian_data, sizeof(T));
  return buffer + sizeof(T);
}

uint8_t *PushBytes(uint8_t *buffer, const void *data, size_t size) {
  std::memcpy(buffer, data, size);
  return buffer + size;
}

uint8_t *Pad(uint8_t *buffer, size_t padding) {
  std::memset(buffer, 0, padding);
  return buffer + padding;
}
}  // namespace

flatbuffers::Offset<MessageHeader> PackRemoteMessage(
    flatbuffers::FlatBufferBuilder *fbb,
    const message_bridge::RemoteMessage *msg, int channel_index,
    const aos::monotonic_clock::time_point monotonic_timestamp_time) {
  logger::MessageHeader::Builder message_header_builder(*fbb);
  // Note: this must match the same order as MessageBridgeServer and
  // PackMessage.  We want identical headers to have identical
  // on-the-wire formats to make comparing them easier.

  message_header_builder.add_channel_index(channel_index);

  message_header_builder.add_queue_index(msg->queue_index());
  message_header_builder.add_monotonic_sent_time(msg->monotonic_sent_time());
  message_header_builder.add_realtime_sent_time(msg->realtime_sent_time());

  message_header_builder.add_monotonic_remote_time(
      msg->monotonic_remote_time());
  message_header_builder.add_realtime_remote_time(msg->realtime_remote_time());
  message_header_builder.add_remote_queue_index(msg->remote_queue_index());

  message_header_builder.add_monotonic_timestamp_time(
      monotonic_timestamp_time.time_since_epoch().count());

  return message_header_builder.Finish();
}

size_t PackRemoteMessageInline(
    uint8_t *buffer, const message_bridge::RemoteMessage *msg,
    int channel_index,
    const aos::monotonic_clock::time_point monotonic_timestamp_time) {
  const flatbuffers::uoffset_t message_size = PackRemoteMessageSize();

  // clang-format off
  // header:
  //   +0x00 | 5C 00 00 00             | UOffset32  | 0x0000005C (92) Loc: +0x5C                | size prefix
  buffer = Push<flatbuffers::uoffset_t>(
      buffer, message_size - sizeof(flatbuffers::uoffset_t));
  //   +0x04 | 20 00 00 00             | UOffset32  | 0x00000020 (32) Loc: +0x24                | offset to root table `aos.logger.MessageHeader`
  buffer = Push<flatbuffers::uoffset_t>(buffer, 0x20);
  //
  // padding:
  //   +0x08 | 00 00 00 00 00 00       | uint8_t[6] | ......                                    | padding
  buffer = Pad(buffer, 6);
  //
  // vtable (aos.logger.MessageHeader):
  //   +0x0E | 16 00                   | uint16_t   | 0x0016 (22)                               | size of this vtable
  buffer = Push<flatbuffers::voffset_t>(buffer, 0x16);
  //   +0x10 | 3C 00                   | uint16_t   | 0x003C (60)                               | size of referring table
  buffer = Push<flatbuffers::voffset_t>(buffer, 0x3c);
  //   +0x12 | 38 00                   | VOffset16  | 0x0038 (56)                               | offset to field `channel_index` (id: 0)
  buffer = Push<flatbuffers::voffset_t>(buffer, 0x38);
  //   +0x14 | 2C 00                   | VOffset16  | 0x002C (44)                               | offset to field `monotonic_sent_time` (id: 1)
  buffer = Push<flatbuffers::voffset_t>(buffer, 0x2c);
  //   +0x16 | 24 00                   | VOffset16  | 0x0024 (36)                               | offset to field `realtime_sent_time` (id: 2)
  buffer = Push<flatbuffers::voffset_t>(buffer, 0x24);
  //   +0x18 | 34 00                   | VOffset16  | 0x0034 (52)                               | offset to field `queue_index` (id: 3)
  buffer = Push<flatbuffers::voffset_t>(buffer, 0x34);
  //   +0x1A | 00 00                   | VOffset16  | 0x0000 (0)                                | offset to field `data` (id: 4) <null> (Vector)
  buffer = Push<flatbuffers::voffset_t>(buffer, 0x00);
  //   +0x1C | 1C 00                   | VOffset16  | 0x001C (28)                               | offset to field `monotonic_remote_time` (id: 5)
  buffer = Push<flatbuffers::voffset_t>(buffer, 0x1c);
  //   +0x1E | 14 00                   | VOffset16  | 0x0014 (20)                               | offset to field `realtime_remote_time` (id: 6)
  buffer = Push<flatbuffers::voffset_t>(buffer, 0x14);
  //   +0x20 | 10 00                   | VOffset16  | 0x0010 (16)                               | offset to field `remote_queue_index` (id: 7)
  buffer = Push<flatbuffers::voffset_t>(buffer, 0x10);
  //   +0x22 | 04 00                   | VOffset16  | 0x0004 (4)                                | offset to field `monotonic_timestamp_time` (id: 8)
  buffer = Push<flatbuffers::voffset_t>(buffer, 0x04);
  //
  // root_table (aos.logger.MessageHeader):
  //   +0x24 | 16 00 00 00             | SOffset32  | 0x00000016 (22) Loc: +0x0E                | offset to vtable
  buffer = Push<flatbuffers::uoffset_t>(buffer, 0x16);
  //   +0x28 | F6 0B D8 11 A4 A8 B1 71 | int64_t    | 0x71B1A8A411D80BF6 (8192514619791117302)  | table field `monotonic_timestamp_time` (Long)
  buffer = Push<int64_t>(buffer,
                         monotonic_timestamp_time.time_since_epoch().count());
  //   +0x30 | 00 00 00 00             | uint8_t[4] | ....                                      | padding
  // TODO(austin): Can we re-arrange the order to ditch the padding?
  // (Answer is yes, but what is the impact elsewhere?  It will change the
  // binary format)
  buffer = Pad(buffer, 4);
  //   +0x34 | 75 00 00 00             | uint32_t   | 0x00000075 (117)                          | table field `remote_queue_index` (UInt)
  buffer = Push<uint32_t>(buffer, msg->remote_queue_index());
  //   +0x38 | AA B0 43 0A 35 BE FA D2 | int64_t    | 0xD2FABE350A43B0AA (-3244071446552268630) | table field `realtime_remote_time` (Long)
  buffer = Push<int64_t>(buffer, msg->realtime_remote_time());
  //   +0x40 | D5 40 30 F3 C1 A7 26 1D | int64_t    | 0x1D26A7C1F33040D5 (2100550727665467605)  | table field `monotonic_remote_time` (Long)
  buffer = Push<int64_t>(buffer, msg->monotonic_remote_time());
  //   +0x48 | 5B 25 32 A1 4A E8 46 CA | int64_t    | 0xCA46E84AA132255B (-3871151422448720549) | table field `realtime_sent_time` (Long)
  buffer = Push<int64_t>(buffer, msg->realtime_sent_time());
  //   +0x50 | 49 7D 45 1F 8C 36 6B A3 | int64_t    | 0xA36B368C1F457D49 (-6671178447571288759) | table field `monotonic_sent_time` (Long)
  buffer = Push<int64_t>(buffer, msg->monotonic_sent_time());
  //   +0x58 | 33 00 00 00             | uint32_t   | 0x00000033 (51)                           | table field `queue_index` (UInt)
  buffer = Push<uint32_t>(buffer, msg->queue_index());
  //   +0x5C | 76 00 00 00             | uint32_t   | 0x00000076 (118)                          | table field `channel_index` (UInt)
  buffer = Push<uint32_t>(buffer, channel_index);
  // clang-format on

  return message_size;
}

flatbuffers::Offset<MessageHeader> PackMessage(
    flatbuffers::FlatBufferBuilder *fbb, const Context &context,
    int channel_index, LogType log_type) {
  flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset;

  switch (log_type) {
    case LogType::kLogMessage:
    case LogType::kLogMessageAndDeliveryTime:
    case LogType::kLogRemoteMessage:
      // Since the timestamps are 8 byte aligned, we are going to end up adding
      // padding in the middle of the message to pad everything out to 8 byte
      // alignment.  That's rather wasteful.  To make things efficient to mmap
      // while reading uncompressed logs, we'd actually rather the message be
      // aligned.  So, force 8 byte alignment (enough to preserve alignment
      // inside the nested message so that we can read it without moving it)
      // here.
      fbb->ForceVectorAlignment(context.size, sizeof(uint8_t), 8);
      data_offset = fbb->CreateVector(
          static_cast<const uint8_t *>(context.data), context.size);
      break;

    case LogType::kLogDeliveryTimeOnly:
      break;
  }

  MessageHeader::Builder message_header_builder(*fbb);
  message_header_builder.add_channel_index(channel_index);

  // These are split out into very explicit serialization calls because the
  // order here changes the order things are written out on the wire, and we
  // want to control and understand it here.  Changing the order can increase
  // the amount of padding bytes in the middle.
  //
  // It is also easier to follow...  And doesn't actually make things much bigger.
  switch (log_type) {
    case LogType::kLogRemoteMessage:
      message_header_builder.add_queue_index(context.remote_queue_index);
      message_header_builder.add_data(data_offset);
      message_header_builder.add_monotonic_sent_time(
          context.monotonic_remote_time.time_since_epoch().count());
      message_header_builder.add_realtime_sent_time(
          context.realtime_remote_time.time_since_epoch().count());
      break;

    case LogType::kLogDeliveryTimeOnly:
      message_header_builder.add_queue_index(context.queue_index);
      message_header_builder.add_monotonic_sent_time(
          context.monotonic_event_time.time_since_epoch().count());
      message_header_builder.add_realtime_sent_time(
          context.realtime_event_time.time_since_epoch().count());
      message_header_builder.add_monotonic_remote_time(
          context.monotonic_remote_time.time_since_epoch().count());
      message_header_builder.add_realtime_remote_time(
          context.realtime_remote_time.time_since_epoch().count());
      message_header_builder.add_remote_queue_index(context.remote_queue_index);
      break;

    case LogType::kLogMessage:
      message_header_builder.add_queue_index(context.queue_index);
      message_header_builder.add_data(data_offset);
      message_header_builder.add_monotonic_sent_time(
          context.monotonic_event_time.time_since_epoch().count());
      message_header_builder.add_realtime_sent_time(
          context.realtime_event_time.time_since_epoch().count());
      break;

    case LogType::kLogMessageAndDeliveryTime:
      message_header_builder.add_queue_index(context.queue_index);
      message_header_builder.add_remote_queue_index(context.remote_queue_index);
      message_header_builder.add_monotonic_sent_time(
          context.monotonic_event_time.time_since_epoch().count());
      message_header_builder.add_realtime_sent_time(
          context.realtime_event_time.time_since_epoch().count());
      message_header_builder.add_monotonic_remote_time(
          context.monotonic_remote_time.time_since_epoch().count());
      message_header_builder.add_realtime_remote_time(
          context.realtime_remote_time.time_since_epoch().count());
      message_header_builder.add_data(data_offset);
      break;
  }

  return message_header_builder.Finish();
}

flatbuffers::uoffset_t PackMessageHeaderSize(LogType log_type) {
  switch (log_type) {
    case LogType::kLogMessage:
      return
          // Root table size + offset.
          sizeof(flatbuffers::uoffset_t) * 2 +
          // 6 padding bytes to pad the header out properly.
          6 +
          // vtable header (size + size of table)
          sizeof(flatbuffers::voffset_t) * 2 +
          // offsets to all the fields.
          sizeof(flatbuffers::voffset_t) * 5 +
          // pointer to vtable
          sizeof(flatbuffers::soffset_t) +
          // pointer to data
          sizeof(flatbuffers::uoffset_t) +
          // realtime_sent_time, monotonic_sent_time
          sizeof(int64_t) * 2 +
          // queue_index, channel_index
          sizeof(uint32_t) * 2;

    case LogType::kLogDeliveryTimeOnly:
      return
          // Root table size + offset.
          sizeof(flatbuffers::uoffset_t) * 2 +
          // 6 padding bytes to pad the header out properly.
          4 +
          // vtable header (size + size of table)
          sizeof(flatbuffers::voffset_t) * 2 +
          // offsets to all the fields.
          sizeof(flatbuffers::voffset_t) * 8 +
          // pointer to vtable
          sizeof(flatbuffers::soffset_t) +
          // remote_queue_index
          sizeof(uint32_t) +
          // realtime_remote_time, monotonic_remote_time, realtime_sent_time,
          // monotonic_sent_time
          sizeof(int64_t) * 4 +
          // queue_index, channel_index
          sizeof(uint32_t) * 2;

    case LogType::kLogMessageAndDeliveryTime:
      return
          // Root table size + offset.
          sizeof(flatbuffers::uoffset_t) * 2 +
          // 4 padding bytes to pad the header out properly.
          4 +
          // vtable header (size + size of table)
          sizeof(flatbuffers::voffset_t) * 2 +
          // offsets to all the fields.
          sizeof(flatbuffers::voffset_t) * 8 +
          // pointer to vtable
          sizeof(flatbuffers::soffset_t) +
          // pointer to data
          sizeof(flatbuffers::uoffset_t) +
          // realtime_remote_time, monotonic_remote_time, realtime_sent_time,
          // monotonic_sent_time
          sizeof(int64_t) * 4 +
          // remote_queue_index, queue_index, channel_index
          sizeof(uint32_t) * 3;

    case LogType::kLogRemoteMessage:
      return
          // Root table size + offset.
          sizeof(flatbuffers::uoffset_t) * 2 +
          // 6 padding bytes to pad the header out properly.
          6 +
          // vtable header (size + size of table)
          sizeof(flatbuffers::voffset_t) * 2 +
          // offsets to all the fields.
          sizeof(flatbuffers::voffset_t) * 5 +
          // pointer to vtable
          sizeof(flatbuffers::soffset_t) +
          // realtime_sent_time, monotonic_sent_time
          sizeof(int64_t) * 2 +
          // pointer to data
          sizeof(flatbuffers::uoffset_t) +
          // queue_index, channel_index
          sizeof(uint32_t) * 2;
  }
  LOG(FATAL);
}

flatbuffers::uoffset_t PackMessageSize(LogType log_type,
                                       const Context &context) {
  static_assert(sizeof(flatbuffers::uoffset_t) == 4u,
                "Update size logic please.");
  const flatbuffers::uoffset_t aligned_data_length =
      ((context.size + 7) & 0xfffffff8u);
  switch (log_type) {
    case LogType::kLogDeliveryTimeOnly:
      return PackMessageHeaderSize(log_type);

    case LogType::kLogMessage:
    case LogType::kLogMessageAndDeliveryTime:
    case LogType::kLogRemoteMessage:
      return PackMessageHeaderSize(log_type) +
             // Vector...
             sizeof(flatbuffers::uoffset_t) + aligned_data_length;
  }
  LOG(FATAL);
}

size_t PackMessageInline(uint8_t *buffer, const Context &context,
                         int channel_index, LogType log_type) {
  const flatbuffers::uoffset_t message_size =
      PackMessageSize(log_type, context);

  buffer = Push<flatbuffers::uoffset_t>(
      buffer, message_size - sizeof(flatbuffers::uoffset_t));

  // Pack all the data in.  This is brittle but easy to change.  Use the
  // InlinePackMessage.Equivilent unit test to verify everything matches.
  switch (log_type) {
    case LogType::kLogMessage:
      // clang-format off
      // header:
      //   +0x00 | 4C 00 00 00             | UOffset32  | 0x0000004C (76) Loc: +0x4C               | size prefix
      //   +0x04 | 18 00 00 00             | UOffset32  | 0x00000018 (24) Loc: +0x1C               | offset to root table `aos.logger.MessageHeader`
      buffer = Push<flatbuffers::uoffset_t>(buffer, 0x18);
      //
      // padding:
      //   +0x08 | 00 00 00 00 00 00       | uint8_t[6] | ......                                   | padding
      buffer = Pad(buffer, 6);
      //
      // vtable (aos.logger.MessageHeader):
      //   +0x0E | 0E 00                   | uint16_t   | 0x000E (14)                              | size of this vtable
      buffer = Push<flatbuffers::voffset_t>(buffer, 0xe);
      //   +0x10 | 20 00                   | uint16_t   | 0x0020 (32)                              | size of referring table
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x20);
      //   +0x12 | 1C 00                   | VOffset16  | 0x001C (28)                              | offset to field `channel_index` (id: 0)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x1c);
      //   +0x14 | 0C 00                   | VOffset16  | 0x000C (12)                              | offset to field `monotonic_sent_time` (id: 1)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x0c);
      //   +0x16 | 04 00                   | VOffset16  | 0x0004 (4)                               | offset to field `realtime_sent_time` (id: 2)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x04);
      //   +0x18 | 18 00                   | VOffset16  | 0x0018 (24)                              | offset to field `queue_index` (id: 3)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x18);
      //   +0x1A | 14 00                   | VOffset16  | 0x0014 (20)                              | offset to field `data` (id: 4)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x14);
      //
      // root_table (aos.logger.MessageHeader):
      //   +0x1C | 0E 00 00 00             | SOffset32  | 0x0000000E (14) Loc: +0x0E               | offset to vtable
      buffer = Push<flatbuffers::uoffset_t>(buffer, 0x0e);
      //   +0x20 | B2 E4 EF 89 19 7D 7F 6F | int64_t    | 0x6F7F7D1989EFE4B2 (8034277808894108850) | table field `realtime_sent_time` (Long)
      buffer = Push<int64_t>(buffer, context.realtime_event_time.time_since_epoch().count());
      //   +0x28 | 86 8D 92 65 FC 79 74 2B | int64_t    | 0x2B7479FC65928D86 (3131261765872160134) | table field `monotonic_sent_time` (Long)
      buffer = Push<int64_t>(buffer, context.monotonic_event_time.time_since_epoch().count());
      //   +0x30 | 0C 00 00 00             | UOffset32  | 0x0000000C (12) Loc: +0x3C               | offset to field `data` (vector)
      buffer = Push<flatbuffers::uoffset_t>(buffer, 0x0c);
      //   +0x34 | 86 00 00 00             | uint32_t   | 0x00000086 (134)                         | table field `queue_index` (UInt)
      buffer = Push<uint32_t>(buffer, context.queue_index);
      //   +0x38 | 71 00 00 00             | uint32_t   | 0x00000071 (113)                         | table field `channel_index` (UInt)
      buffer = Push<uint32_t>(buffer, channel_index);
      //
      // vector (aos.logger.MessageHeader.data):
      //   +0x3C | 0E 00 00 00             | uint32_t   | 0x0000000E (14)                          | length of vector (# items)
      buffer = Push<flatbuffers::uoffset_t>(buffer, context.size);
      //   +0x40 | FF                      | uint8_t    | 0xFF (255)                               | value[0]
      //   +0x41 | B8                      | uint8_t    | 0xB8 (184)                               | value[1]
      //   +0x42 | EE                      | uint8_t    | 0xEE (238)                               | value[2]
      //   +0x43 | 00                      | uint8_t    | 0x00 (0)                                 | value[3]
      //   +0x44 | 20                      | uint8_t    | 0x20 (32)                                | value[4]
      //   +0x45 | 4D                      | uint8_t    | 0x4D (77)                                | value[5]
      //   +0x46 | FF                      | uint8_t    | 0xFF (255)                               | value[6]
      //   +0x47 | 25                      | uint8_t    | 0x25 (37)                                | value[7]
      //   +0x48 | 3C                      | uint8_t    | 0x3C (60)                                | value[8]
      //   +0x49 | 17                      | uint8_t    | 0x17 (23)                                | value[9]
      //   +0x4A | 65                      | uint8_t    | 0x65 (101)                               | value[10]
      //   +0x4B | 2F                      | uint8_t    | 0x2F (47)                                | value[11]
      //   +0x4C | 63                      | uint8_t    | 0x63 (99)                                | value[12]
      //   +0x4D | 58                      | uint8_t    | 0x58 (88)                                | value[13]
      buffer = PushBytes(buffer, context.data, context.size);
      //
      // padding:
      //   +0x4E | 00 00                   | uint8_t[2] | ..                                       | padding
      buffer = Pad(buffer, ((context.size + 7) & 0xfffffff8u) - context.size);
      // clang-format on
      break;

    case LogType::kLogDeliveryTimeOnly:
      // clang-format off
      // header:
      //   +0x00 | 4C 00 00 00             | UOffset32  | 0x0000004C (76) Loc: +0x4C                | size prefix
      //   +0x04 | 1C 00 00 00             | UOffset32  | 0x0000001C (28) Loc: +0x20                | offset to root table `aos.logger.MessageHeader`
      buffer = Push<flatbuffers::uoffset_t>(buffer, 0x1c);
      //
      // padding:
      //   +0x08 | 00 00 00 00             | uint8_t[4] | ....                                      | padding
      buffer = Pad(buffer, 4);
      //
      // vtable (aos.logger.MessageHeader):
      //   +0x0C | 14 00                   | uint16_t   | 0x0014 (20)                               | size of this vtable
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x14);
      //   +0x0E | 30 00                   | uint16_t   | 0x0030 (48)                               | size of referring table
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x30);
      //   +0x10 | 2C 00                   | VOffset16  | 0x002C (44)                               | offset to field `channel_index` (id: 0)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x2c);
      //   +0x12 | 20 00                   | VOffset16  | 0x0020 (32)                               | offset to field `monotonic_sent_time` (id: 1)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x20);
      //   +0x14 | 18 00                   | VOffset16  | 0x0018 (24)                               | offset to field `realtime_sent_time` (id: 2)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x18);
      //   +0x16 | 28 00                   | VOffset16  | 0x0028 (40)                               | offset to field `queue_index` (id: 3)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x28);
      //   +0x18 | 00 00                   | VOffset16  | 0x0000 (0)                                | offset to field `data` (id: 4) <null> (Vector)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x00);
      //   +0x1A | 10 00                   | VOffset16  | 0x0010 (16)                               | offset to field `monotonic_remote_time` (id: 5)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x10);
      //   +0x1C | 08 00                   | VOffset16  | 0x0008 (8)                                | offset to field `realtime_remote_time` (id: 6)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x08);
      //   +0x1E | 04 00                   | VOffset16  | 0x0004 (4)                                | offset to field `remote_queue_index` (id: 7)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x04);
      //
      // root_table (aos.logger.MessageHeader):
      //   +0x20 | 14 00 00 00             | SOffset32  | 0x00000014 (20) Loc: +0x0C                | offset to vtable
      buffer = Push<flatbuffers::uoffset_t>(buffer, 0x14);
      //   +0x24 | 69 00 00 00             | uint32_t   | 0x00000069 (105)                          | table field `remote_queue_index` (UInt)
      buffer = Push<uint32_t>(buffer, context.remote_queue_index);
      //   +0x28 | C6 85 F1 AB 83 B5 CD EB | int64_t    | 0xEBCDB583ABF185C6 (-1455307527440726586) | table field `realtime_remote_time` (Long)
      buffer = Push<int64_t>(buffer, context.realtime_remote_time.time_since_epoch().count());
      //   +0x30 | 47 24 D3 97 1E 42 2D 99 | int64_t    | 0x992D421E97D32447 (-7409193112790948793) | table field `monotonic_remote_time` (Long)
      buffer = Push<int64_t>(buffer, context.monotonic_remote_time.time_since_epoch().count());
      //   +0x38 | C8 B9 A7 AB 79 F2 CD 60 | int64_t    | 0x60CDF279ABA7B9C8 (6975498002251626952)  | table field `realtime_sent_time` (Long)
      buffer = Push<int64_t>(buffer, context.realtime_event_time.time_since_epoch().count());
      //   +0x40 | EA 8F 2A 0F AF 01 7A AB | int64_t    | 0xAB7A01AF0F2A8FEA (-6090553694679822358) | table field `monotonic_sent_time` (Long)
      buffer = Push<int64_t>(buffer, context.monotonic_event_time.time_since_epoch().count());
      //   +0x48 | F5 00 00 00             | uint32_t   | 0x000000F5 (245)                          | table field `queue_index` (UInt)
      buffer = Push<uint32_t>(buffer, context.queue_index);
      //   +0x4C | 88 00 00 00             | uint32_t   | 0x00000088 (136)                          | table field `channel_index` (UInt)
      buffer = Push<uint32_t>(buffer, channel_index);

      // clang-format on
      break;

    case LogType::kLogMessageAndDeliveryTime:
      // clang-format off
      // header:
      //   +0x00 | 5C 00 00 00             | UOffset32  | 0x0000005C (92) Loc: +0x5C                | size prefix
      //   +0x04 | 1C 00 00 00             | UOffset32  | 0x0000001C (28) Loc: +0x20                | offset to root table `aos.logger.MessageHeader`
      buffer = Push<flatbuffers::uoffset_t>(buffer, 0x1c);
      //
      // padding:
      //   +0x08 | 00 00 00 00             | uint8_t[4] | ....                                      | padding
      buffer = Pad(buffer, 4);
      //
      // vtable (aos.logger.MessageHeader):
      //   +0x0C | 14 00                   | uint16_t   | 0x0014 (20)                               | size of this vtable
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x14);
      //   +0x0E | 34 00                   | uint16_t   | 0x0034 (52)                               | size of referring table
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x34);
      //   +0x10 | 30 00                   | VOffset16  | 0x0030 (48)                               | offset to field `channel_index` (id: 0)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x30);
      //   +0x12 | 20 00                   | VOffset16  | 0x0020 (32)                               | offset to field `monotonic_sent_time` (id: 1)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x20);
      //   +0x14 | 18 00                   | VOffset16  | 0x0018 (24)                               | offset to field `realtime_sent_time` (id: 2)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x18);
      //   +0x16 | 2C 00                   | VOffset16  | 0x002C (44)                               | offset to field `queue_index` (id: 3)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x2c);
      //   +0x18 | 04 00                   | VOffset16  | 0x0004 (4)                                | offset to field `data` (id: 4)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x04);
      //   +0x1A | 10 00                   | VOffset16  | 0x0010 (16)                               | offset to field `monotonic_remote_time` (id: 5)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x10);
      //   +0x1C | 08 00                   | VOffset16  | 0x0008 (8)                                | offset to field `realtime_remote_time` (id: 6)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x08);
      //   +0x1E | 28 00                   | VOffset16  | 0x0028 (40)                               | offset to field `remote_queue_index` (id: 7)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x28);
      //
      // root_table (aos.logger.MessageHeader):
      //   +0x20 | 14 00 00 00             | SOffset32  | 0x00000014 (20) Loc: +0x0C                | offset to vtable
      buffer = Push<flatbuffers::uoffset_t>(buffer, 0x14);
      //   +0x24 | 30 00 00 00             | UOffset32  | 0x00000030 (48) Loc: +0x54                | offset to field `data` (vector)
      buffer = Push<flatbuffers::uoffset_t>(buffer, 0x30);
      //   +0x28 | C4 C8 87 BF 40 6C 1F 29 | int64_t    | 0x291F6C40BF87C8C4 (2963206105180129476)  | table field `realtime_remote_time` (Long)
      buffer = Push<int64_t>(buffer, context.realtime_remote_time.time_since_epoch().count());
      //   +0x30 | 0F 00 26 FD D2 6D C0 1F | int64_t    | 0x1FC06DD2FD26000F (2287949363661897743)  | table field `monotonic_remote_time` (Long)
      buffer = Push<int64_t>(buffer, context.monotonic_remote_time.time_since_epoch().count());
      //   +0x38 | 29 75 09 C0 73 73 BF 88 | int64_t    | 0x88BF7373C0097529 (-8593022623019338455) | table field `realtime_sent_time` (Long)
      buffer = Push<int64_t>(buffer, context.realtime_event_time.time_since_epoch().count());
      //   +0x40 | 6D 8A AE 04 50 25 9C E9 | int64_t    | 0xE99C255004AE8A6D (-1613373540899321235) | table field `monotonic_sent_time` (Long)
      buffer = Push<int64_t>(buffer, context.monotonic_event_time.time_since_epoch().count());
      //   +0x48 | 47 00 00 00             | uint32_t   | 0x00000047 (71)                           | table field `remote_queue_index` (UInt)
      buffer = Push<uint32_t>(buffer, context.remote_queue_index);
      //   +0x4C | 4C 00 00 00             | uint32_t   | 0x0000004C (76)                           | table field `queue_index` (UInt)
      buffer = Push<uint32_t>(buffer, context.queue_index);
      //   +0x50 | 72 00 00 00             | uint32_t   | 0x00000072 (114)                          | table field `channel_index` (UInt)
      buffer = Push<uint32_t>(buffer, channel_index);
      //
      // vector (aos.logger.MessageHeader.data):
      //   +0x54 | 07 00 00 00             | uint32_t   | 0x00000007 (7)                            | length of vector (# items)
      buffer = Push<flatbuffers::uoffset_t>(buffer, context.size);
      //   +0x58 | B1                      | uint8_t    | 0xB1 (177)                                | value[0]
      //   +0x59 | 4A                      | uint8_t    | 0x4A (74)                                 | value[1]
      //   +0x5A | 50                      | uint8_t    | 0x50 (80)                                 | value[2]
      //   +0x5B | 24                      | uint8_t    | 0x24 (36)                                 | value[3]
      //   +0x5C | AF                      | uint8_t    | 0xAF (175)                                | value[4]
      //   +0x5D | C8                      | uint8_t    | 0xC8 (200)                                | value[5]
      //   +0x5E | D5                      | uint8_t    | 0xD5 (213)                                | value[6]
      buffer = PushBytes(buffer, context.data, context.size);
      //
      // padding:
      //   +0x5F | 00                      | uint8_t[1] | .                                         | padding
      buffer = Pad(buffer, ((context.size + 7) & 0xfffffff8u) - context.size);
      // clang-format on

      break;

    case LogType::kLogRemoteMessage:
      // This is the message we need to recreate.
      //
      // clang-format off
      // header:
      //   +0x00 | 5C 00 00 00             | UOffset32  | 0x0000005C (92) Loc: +0x5C                | size prefix
      //   +0x04 | 18 00 00 00             | UOffset32  | 0x00000018 (24) Loc: +0x1C                | offset to root table `aos.logger.MessageHeader`
      buffer = Push<flatbuffers::uoffset_t>(buffer, 0x18);
      //
      // padding:
      //   +0x08 | 00 00 00 00 00 00       | uint8_t[6] | ......                                    | padding
      buffer = Pad(buffer, 6);
      //
      // vtable (aos.logger.MessageHeader):
      //   +0x0E | 0E 00                   | uint16_t   | 0x000E (14)                               | size of this vtable
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x0e);
      //   +0x10 | 20 00                   | uint16_t   | 0x0020 (32)                               | size of referring table
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x20);
      //   +0x12 | 1C 00                   | VOffset16  | 0x001C (28)                               | offset to field `channel_index` (id: 0)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x1c);
      //   +0x14 | 0C 00                   | VOffset16  | 0x000C (12)                               | offset to field `monotonic_sent_time` (id: 1)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x0c);
      //   +0x16 | 04 00                   | VOffset16  | 0x0004 (4)                                | offset to field `realtime_sent_time` (id: 2)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x04);
      //   +0x18 | 18 00                   | VOffset16  | 0x0018 (24)                               | offset to field `queue_index` (id: 3)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x18);
      //   +0x1A | 14 00                   | VOffset16  | 0x0014 (20)                               | offset to field `data` (id: 4)
      buffer = Push<flatbuffers::voffset_t>(buffer, 0x14);
      //
      // root_table (aos.logger.MessageHeader):
      //   +0x1C | 0E 00 00 00             | SOffset32  | 0x0000000E (14) Loc: +0x0E                | offset to vtable
      buffer = Push<flatbuffers::uoffset_t>(buffer, 0x0E);
      //   +0x20 | D8 96 32 1A A0 D3 23 BB | int64_t    | 0xBB23D3A01A3296D8 (-4961889679844403496) | table field `realtime_sent_time` (Long)
      buffer = Push<int64_t>(buffer, context.realtime_remote_time.time_since_epoch().count());
      //   +0x28 | 2E 5D 23 B3 BE 84 CF C2 | int64_t    | 0xC2CF84BEB3235D2E (-4409159555588334290) | table field `monotonic_sent_time` (Long)
      buffer = Push<int64_t>(buffer, context.monotonic_remote_time.time_since_epoch().count());
      //   +0x30 | 0C 00 00 00             | UOffset32  | 0x0000000C (12) Loc: +0x3C                | offset to field `data` (vector)
      buffer = Push<flatbuffers::uoffset_t>(buffer, 0x0C);
      //   +0x34 | 69 00 00 00             | uint32_t   | 0x00000069 (105)                          | table field `queue_index` (UInt)
      buffer = Push<uint32_t>(buffer, context.remote_queue_index);
      //   +0x38 | F3 00 00 00             | uint32_t   | 0x000000F3 (243)                          | table field `channel_index` (UInt)
      buffer = Push<uint32_t>(buffer, channel_index);
      //
      // vector (aos.logger.MessageHeader.data):
      //   +0x3C | 1A 00 00 00             | uint32_t   | 0x0000001A (26)                           | length of vector (# items)
      buffer = Push<flatbuffers::uoffset_t>(buffer, context.size);
      //   +0x40 | 38                      | uint8_t    | 0x38 (56)                                 | value[0]
      //   +0x41 | 1A                      | uint8_t    | 0x1A (26)                                 | value[1]
      // ...
      //   +0x58 | 90                      | uint8_t    | 0x90 (144)                                | value[24]
      //   +0x59 | 92                      | uint8_t    | 0x92 (146)                                | value[25]
      buffer = PushBytes(buffer, context.data, context.size);
      //
      // padding:
      //   +0x5A | 00 00 00 00 00 00       | uint8_t[6] | ......                                    | padding
      buffer = Pad(buffer, ((context.size + 7) & 0xfffffff8u) - context.size);
      // clang-format on
  }

  return message_size;
}

SpanReader::SpanReader(std::string_view filename, bool quiet)
    : filename_(filename) {
  decoder_ = std::make_unique<DummyDecoder>(filename);

  static constexpr std::string_view kXz = ".xz";
  static constexpr std::string_view kSnappy = SnappyDecoder::kExtension;
  if (filename.substr(filename.size() - kXz.size()) == kXz) {
#if ENABLE_LZMA
    decoder_ =
        std::make_unique<ThreadedLzmaDecoder>(std::move(decoder_), quiet);
#else
    (void)quiet;
    LOG(FATAL) << "Reading xz-compressed files not supported on this platform";
#endif
  } else if (filename.substr(filename.size() - kSnappy.size()) == kSnappy) {
    decoder_ = std::make_unique<SnappyDecoder>(std::move(decoder_));
  }
}

absl::Span<const uint8_t> SpanReader::PeekMessage() {
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

  return absl::Span<const uint8_t>(data_ptr, data_size);
}

void SpanReader::ConsumeMessage() {
  size_t consumed_size =
      flatbuffers::GetPrefixedSize(data_.data() + consumed_data_) +
      sizeof(flatbuffers::uoffset_t);
  consumed_data_ += consumed_size;
  total_consumed_ += consumed_size;
}

absl::Span<const uint8_t> SpanReader::ReadMessage() {
  absl::Span<const uint8_t> result = PeekMessage();
  if (result != absl::Span<const uint8_t>()) {
    ConsumeMessage();
  } else {
    is_finished_ = true;
  }
  return result;
}

bool SpanReader::ReadBlock() {
  // This is the amount of data we grab at a time. Doing larger chunks minimizes
  // syscalls and helps decompressors batch things more efficiently.
  constexpr size_t kReadSize = 256 * 1024;

  // Strip off any unused data at the front.
  if (consumed_data_ != 0) {
    data_.erase_front(consumed_data_);
    consumed_data_ = 0;
  }

  const size_t starting_size = data_.size();

  // This should automatically grow the backing store.  It won't shrink if we
  // get a small chunk later.  This reduces allocations when we want to append
  // more data.
  data_.resize(starting_size + kReadSize);

  const size_t count =
      decoder_->Read(data_.begin() + starting_size, data_.end());
  data_.resize(starting_size + count);
  if (count == 0) {
    return false;
  }

  total_read_ += count;

  return true;
}

std::optional<SizePrefixedFlatbufferVector<LogFileHeader>> ReadHeader(
    SpanReader *span_reader) {
  absl::Span<const uint8_t> config_data = span_reader->ReadMessage();

  // Make sure something was read.
  if (config_data == absl::Span<const uint8_t>()) {
    return std::nullopt;
  }

  // And copy the config so we have it forever, removing the size prefix.
  SizePrefixedFlatbufferVector<LogFileHeader> result(config_data);
  if (!result.Verify()) {
    return std::nullopt;
  }

  if (FLAGS_workaround_double_headers) {
    while (true) {
      absl::Span<const uint8_t> maybe_header_data = span_reader->PeekMessage();
      if (maybe_header_data == absl::Span<const uint8_t>()) {
        break;
      }

      aos::SizePrefixedFlatbufferSpan<aos::logger::LogFileHeader> maybe_header(
          maybe_header_data);
      if (maybe_header.Verify()) {
        LOG(WARNING) << "Found duplicate LogFileHeader in "
                     << span_reader->filename();
        ResizeableBuffer header_data_copy;
        header_data_copy.resize(maybe_header_data.size());
        memcpy(header_data_copy.data(), maybe_header_data.begin(),
               header_data_copy.size());
        result = SizePrefixedFlatbufferVector<LogFileHeader>(
            std::move(header_data_copy));

        span_reader->ConsumeMessage();
      } else {
        break;
      }
    }
  }
  return result;
}

std::optional<SizePrefixedFlatbufferVector<LogFileHeader>> ReadHeader(
    std::string_view filename) {
  SpanReader span_reader(filename);
  return ReadHeader(&span_reader);
}

std::optional<SizePrefixedFlatbufferVector<MessageHeader>> ReadNthMessage(
    std::string_view filename, size_t n) {
  SpanReader span_reader(filename);
  absl::Span<const uint8_t> data_span = span_reader.ReadMessage();
  for (size_t i = 0; i < n + 1; ++i) {
    data_span = span_reader.ReadMessage();

    // Make sure something was read.
    if (data_span == absl::Span<const uint8_t>()) {
      return std::nullopt;
    }
  }

  // And copy the config so we have it forever, removing the size prefix.
  SizePrefixedFlatbufferVector<MessageHeader> result(data_span);
  if (!result.Verify()) {
    return std::nullopt;
  }
  return result;
}

MessageReader::MessageReader(std::string_view filename)
    : span_reader_(filename),
      raw_log_file_header_(
          SizePrefixedFlatbufferVector<LogFileHeader>::Empty()) {
  set_crash_on_corrupt_message_flag(FLAGS_crash_on_corrupt_message);
  set_ignore_corrupt_messages_flag(FLAGS_ignore_corrupt_messages);

  std::optional<SizePrefixedFlatbufferVector<LogFileHeader>>
      raw_log_file_header = ReadHeader(&span_reader_);

  // Make sure something was read.
  CHECK(raw_log_file_header) << ": Failed to read header from: " << filename;

  raw_log_file_header_ = std::move(*raw_log_file_header);

  CHECK(raw_log_file_header_.Verify()) << "Log file header is corrupted";

  total_verified_before_ = span_reader_.TotalConsumed();

  max_out_of_order_duration_ =
      FLAGS_max_out_of_order > 0
          ? chrono::duration_cast<chrono::nanoseconds>(
                chrono::duration<double>(FLAGS_max_out_of_order))
          : chrono::nanoseconds(log_file_header()->max_out_of_order_duration());

  VLOG(1) << "Opened " << filename << " as node "
          << FlatbufferToJson(log_file_header()->node());
}

std::shared_ptr<UnpackedMessageHeader> MessageReader::ReadMessage() {
  absl::Span<const uint8_t> msg_data = span_reader_.ReadMessage();
  if (msg_data == absl::Span<const uint8_t>()) {
    if (is_corrupted()) {
      LOG(ERROR) << "Total corrupted volumes: before = "
                 << total_verified_before_
                 << " | corrupted = " << total_corrupted_
                 << " | during = " << total_verified_during_
                 << " | after = " << total_verified_after_ << std::endl;
    }

    if (span_reader_.IsIncomplete()) {
      LOG(ERROR) << "Unable to access some messages in " << filename() << " : "
                 << span_reader_.TotalRead() << " bytes read, "
                 << span_reader_.TotalConsumed() << " bytes usable."
                 << std::endl;
    }
    return nullptr;
  }

  SizePrefixedFlatbufferSpan<MessageHeader> msg(msg_data);

  if (crash_on_corrupt_message_flag_) {
    CHECK(msg.Verify()) << "Corrupted message at offset "
                        << total_verified_before_ << " found within "
                        << filename()
                        << "; set --nocrash_on_corrupt_message to see summary;"
                        << " also set --ignore_corrupt_messages to process"
                        << " anyway";

  } else if (!msg.Verify()) {
    LOG(ERROR) << "Corrupted message at offset " << total_verified_before_
               << " from " << filename() << std::endl;

    total_corrupted_ += msg_data.size();

    while (true) {
      absl::Span<const uint8_t> msg_data = span_reader_.ReadMessage();

      if (msg_data == absl::Span<const uint8_t>()) {
        if (!ignore_corrupt_messages_flag_) {
          LOG(ERROR) << "Total corrupted volumes: before = "
                     << total_verified_before_
                     << " | corrupted = " << total_corrupted_
                     << " | during = " << total_verified_during_
                     << " | after = " << total_verified_after_ << std::endl;

          if (span_reader_.IsIncomplete()) {
            LOG(ERROR) << "Unable to access some messages in " << filename()
                       << " : " << span_reader_.TotalRead() << " bytes read, "
                       << span_reader_.TotalConsumed() << " bytes usable."
                       << std::endl;
          }
          return nullptr;
        }
        break;
      }

      SizePrefixedFlatbufferSpan<MessageHeader> next_msg(msg_data);

      if (!next_msg.Verify()) {
        total_corrupted_ += msg_data.size();
        total_verified_during_ += total_verified_after_;
        total_verified_after_ = 0;

      } else {
        total_verified_after_ += msg_data.size();
        if (ignore_corrupt_messages_flag_) {
          msg = next_msg;
          break;
        }
      }
    }
  }

  if (is_corrupted()) {
    total_verified_after_ += msg_data.size();
  } else {
    total_verified_before_ += msg_data.size();
  }

  auto result = UnpackedMessageHeader::MakeMessage(msg.message());

  const monotonic_clock::time_point timestamp = result->monotonic_sent_time;

  newest_timestamp_ = std::max(newest_timestamp_, timestamp);

  if (VLOG_IS_ON(3)) {
    VLOG(3) << "Read from " << filename() << " data " << FlatbufferToJson(msg);
  } else if (VLOG_IS_ON(2)) {
    SizePrefixedFlatbufferVector<MessageHeader> msg_copy = msg;
    msg_copy.mutable_message()->clear_data();
    VLOG(2) << "Read from " << filename() << " data "
            << FlatbufferToJson(msg_copy);
  }

  return result;
}

std::shared_ptr<UnpackedMessageHeader> UnpackedMessageHeader::MakeMessage(
    const MessageHeader &message) {
  const size_t data_size = message.has_data() ? message.data()->size() : 0;

  UnpackedMessageHeader *const unpacked_message =
      reinterpret_cast<UnpackedMessageHeader *>(
          malloc(sizeof(UnpackedMessageHeader) + data_size +
                 kChannelDataAlignment - 1));

  CHECK(message.has_channel_index());
  CHECK(message.has_monotonic_sent_time());

  absl::Span<uint8_t> span;
  if (data_size > 0) {
    span =
        absl::Span<uint8_t>(reinterpret_cast<uint8_t *>(RoundChannelData(
                                &unpacked_message->actual_data[0], data_size)),
                            data_size);
  }

  std::optional<aos::monotonic_clock::time_point> monotonic_remote_time;
  if (message.has_monotonic_remote_time()) {
    monotonic_remote_time = aos::monotonic_clock::time_point(
        std::chrono::nanoseconds(message.monotonic_remote_time()));
  }
  std::optional<realtime_clock::time_point> realtime_remote_time;
  if (message.has_realtime_remote_time()) {
    realtime_remote_time = realtime_clock::time_point(
        chrono::nanoseconds(message.realtime_remote_time()));
  }

  std::optional<uint32_t> remote_queue_index;
  if (message.has_remote_queue_index()) {
    remote_queue_index = message.remote_queue_index();
  }

  new (unpacked_message) UnpackedMessageHeader{
      .channel_index = message.channel_index(),
      .monotonic_sent_time = monotonic_clock::time_point(
          chrono::nanoseconds(message.monotonic_sent_time())),
      .realtime_sent_time = realtime_clock::time_point(
          chrono::nanoseconds(message.realtime_sent_time())),
      .queue_index = message.queue_index(),
      .monotonic_remote_time = monotonic_remote_time,
      .realtime_remote_time = realtime_remote_time,
      .remote_queue_index = remote_queue_index,
      .monotonic_timestamp_time = monotonic_clock::time_point(
          std::chrono::nanoseconds(message.monotonic_timestamp_time())),
      .has_monotonic_timestamp_time = message.has_monotonic_timestamp_time(),
      .span = span};

  if (data_size > 0) {
    memcpy(span.data(), message.data()->data(), data_size);
  }

  return std::shared_ptr<UnpackedMessageHeader>(unpacked_message,
                                                &DestroyAndFree);
}

PartsMessageReader::PartsMessageReader(LogParts log_parts)
    : parts_(std::move(log_parts)), message_reader_(parts_.parts[0]) {
  if (parts_.parts.size() >= 2) {
    next_message_reader_.emplace(parts_.parts[1]);
  }
  ComputeBootCounts();
}

void PartsMessageReader::ComputeBootCounts() {
  boot_counts_.assign(configuration::NodesCount(parts_.config.get()),
                      std::nullopt);

  // We have 3 vintages of log files with different amounts of information.
  if (log_file_header()->has_boot_uuids()) {
    // The new hotness with the boots explicitly listed out.  We can use the log
    // file header to compute the boot count of all relevant nodes.
    CHECK_EQ(log_file_header()->boot_uuids()->size(), boot_counts_.size());
    size_t node_index = 0;
    for (const flatbuffers::String *boot_uuid :
         *log_file_header()->boot_uuids()) {
      CHECK(parts_.boots);
      if (boot_uuid->size() != 0) {
        auto it = parts_.boots->boot_count_map.find(boot_uuid->str());
        if (it != parts_.boots->boot_count_map.end()) {
          boot_counts_[node_index] = it->second;
        }
      } else if (parts().boots->boots[node_index].size() == 1u) {
        boot_counts_[node_index] = 0;
      }
      ++node_index;
    }
  } else {
    // Older multi-node logs which are guarenteed to have UUIDs logged, or
    // single node log files with boot UUIDs in the header.  We only know how to
    // order certain boots in certain circumstances.
    if (configuration::MultiNode(parts_.config.get()) || parts_.boots) {
      for (size_t node_index = 0; node_index < boot_counts_.size();
           ++node_index) {
        CHECK(parts_.boots);
        if (parts().boots->boots[node_index].size() == 1u) {
          boot_counts_[node_index] = 0;
        }
      }
    } else {
      // Really old single node logs without any UUIDs.  They can't reboot.
      CHECK_EQ(boot_counts_.size(), 1u);
      boot_counts_[0] = 0u;
    }
  }
}

std::shared_ptr<UnpackedMessageHeader> PartsMessageReader::ReadMessage() {
  while (!done_) {
    std::shared_ptr<UnpackedMessageHeader> message =
        message_reader_.ReadMessage();
    if (message) {
      newest_timestamp_ = message_reader_.newest_timestamp();
      const monotonic_clock::time_point monotonic_sent_time =
          message->monotonic_sent_time;

      // TODO(austin): Does this work with startup?  Might need to use the
      // start time.
      // TODO(austin): Does this work with startup when we don't know the
      // remote start time too?  Look at one of those logs to compare.
      if (monotonic_sent_time >
          parts_.monotonic_start_time + max_out_of_order_duration()) {
        after_start_ = true;
      }
      if (after_start_) {
        CHECK_GE(monotonic_sent_time,
                 newest_timestamp_ - max_out_of_order_duration())
            << ": Max out of order of " << max_out_of_order_duration().count()
            << "ns exceeded. " << parts_ << ", start time is "
            << parts_.monotonic_start_time << " currently reading "
            << filename();
      }
      return message;
    }
    NextLog();
  }
  newest_timestamp_ = monotonic_clock::max_time;
  return nullptr;
}

void PartsMessageReader::NextLog() {
  if (next_part_index_ == parts_.parts.size()) {
    CHECK(!next_message_reader_);
    done_ = true;
    return;
  }
  CHECK(next_message_reader_);
  message_reader_ = std::move(*next_message_reader_);
  ComputeBootCounts();
  if (next_part_index_ + 1 < parts_.parts.size()) {
    next_message_reader_.emplace(parts_.parts[next_part_index_ + 1]);
  } else {
    next_message_reader_.reset();
  }
  ++next_part_index_;
}

bool Message::operator<(const Message &m2) const {
  CHECK_EQ(this->timestamp.boot, m2.timestamp.boot);

  if (this->timestamp.time < m2.timestamp.time) {
    return true;
  } else if (this->timestamp.time > m2.timestamp.time) {
    return false;
  }

  if (this->channel_index < m2.channel_index) {
    return true;
  } else if (this->channel_index > m2.channel_index) {
    return false;
  }

  return this->queue_index < m2.queue_index;
}

bool Message::operator>=(const Message &m2) const { return !(*this < m2); }
bool Message::operator==(const Message &m2) const {
  CHECK_EQ(this->timestamp.boot, m2.timestamp.boot);

  return timestamp.time == m2.timestamp.time &&
         channel_index == m2.channel_index && queue_index == m2.queue_index;
}

std::ostream &operator<<(std::ostream &os, const UnpackedMessageHeader &m) {
  os << "{.channel_index=" << m.channel_index
     << ", .monotonic_sent_time=" << m.monotonic_sent_time
     << ", .realtime_sent_time=" << m.realtime_sent_time
     << ", .queue_index=" << m.queue_index;
  if (m.monotonic_remote_time) {
    os << ", .monotonic_remote_time=" << *m.monotonic_remote_time;
  }
  os << ", .realtime_remote_time=";
  PrintOptionalOrNull(&os, m.realtime_remote_time);
  os << ", .remote_queue_index=";
  PrintOptionalOrNull(&os, m.remote_queue_index);
  if (m.has_monotonic_timestamp_time) {
    os << ", .monotonic_timestamp_time=" << m.monotonic_timestamp_time;
  }
  os << "}";
  return os;
}

std::ostream &operator<<(std::ostream &os, const Message &m) {
  os << "{.channel_index=" << m.channel_index
     << ", .queue_index=" << m.queue_index << ", .timestamp=" << m.timestamp;
  if (m.data != nullptr) {
    if (m.data->remote_queue_index.has_value()) {
      os << ", .remote_queue_index=" << *m.data->remote_queue_index;
    }
    if (m.data->monotonic_remote_time.has_value()) {
      os << ", .monotonic_remote_time=" << *m.data->monotonic_remote_time;
    }
    os << ", .data=" << m.data;
  }
  os << "}";
  return os;
}

std::ostream &operator<<(std::ostream &os, const TimestampedMessage &m) {
  os << "{.channel_index=" << m.channel_index
     << ", .queue_index=" << m.queue_index
     << ", .monotonic_event_time=" << m.monotonic_event_time
     << ", .realtime_event_time=" << m.realtime_event_time;
  if (m.remote_queue_index != BootQueueIndex::Invalid()) {
    os << ", .remote_queue_index=" << m.remote_queue_index;
  }
  if (m.monotonic_remote_time != BootTimestamp::min_time()) {
    os << ", .monotonic_remote_time=" << m.monotonic_remote_time;
  }
  if (m.realtime_remote_time != realtime_clock::min_time) {
    os << ", .realtime_remote_time=" << m.realtime_remote_time;
  }
  if (m.monotonic_timestamp_time != BootTimestamp::min_time()) {
    os << ", .monotonic_timestamp_time=" << m.monotonic_timestamp_time;
  }
  if (m.data != nullptr) {
    os << ", .data=" << *m.data;
  } else {
    os << ", .data=nullptr";
  }
  os << "}";
  return os;
}

LogPartsSorter::LogPartsSorter(LogParts log_parts)
    : parts_message_reader_(log_parts),
      source_node_index_(configuration::SourceNodeIndex(parts().config.get())) {
}

Message *LogPartsSorter::Front() {
  // Queue up data until enough data has been queued that the front message is
  // sorted enough to be safe to pop.  This may do nothing, so we should make
  // sure the nothing path is checked quickly.
  if (sorted_until() != monotonic_clock::max_time) {
    while (true) {
      if (!messages_.empty() &&
          messages_.begin()->timestamp.time < sorted_until() &&
          sorted_until() >= monotonic_start_time()) {
        break;
      }

      std::shared_ptr<UnpackedMessageHeader> m =
          parts_message_reader_.ReadMessage();
      // No data left, sorted forever, work through what is left.
      if (!m) {
        sorted_until_ = monotonic_clock::max_time;
        break;
      }

      size_t monotonic_timestamp_boot = 0;
      if (m->has_monotonic_timestamp_time) {
        monotonic_timestamp_boot = parts().logger_boot_count;
      }
      size_t monotonic_remote_boot = 0xffffff;

      if (m->monotonic_remote_time.has_value()) {
        const Node *node =
            parts().config->nodes()->Get(source_node_index_[m->channel_index]);

        std::optional<size_t> boot = parts_message_reader_.boot_count(
            source_node_index_[m->channel_index]);
        CHECK(boot) << ": Failed to find boot for node " << MaybeNodeName(node)
                    << ", with index " << source_node_index_[m->channel_index];
        monotonic_remote_boot = *boot;
      }

      messages_.insert(
          Message{.channel_index = m->channel_index,
                  .queue_index = BootQueueIndex{.boot = parts().boot_count,
                                                .index = m->queue_index},
                  .timestamp = BootTimestamp{.boot = parts().boot_count,
                                             .time = m->monotonic_sent_time},
                  .monotonic_remote_boot = monotonic_remote_boot,
                  .monotonic_timestamp_boot = monotonic_timestamp_boot,
                  .data = std::move(m)});

      // Now, update sorted_until_ to match the new message.
      if (parts_message_reader_.newest_timestamp() >
          monotonic_clock::min_time +
              parts_message_reader_.max_out_of_order_duration()) {
        sorted_until_ = parts_message_reader_.newest_timestamp() -
                        parts_message_reader_.max_out_of_order_duration();
      } else {
        sorted_until_ = monotonic_clock::min_time;
      }
    }
  }

  // Now that we have enough data queued, return a pointer to the oldest piece
  // of data if it exists.
  if (messages_.empty()) {
    last_message_time_ = monotonic_clock::max_time;
    return nullptr;
  }

  CHECK_GE(messages_.begin()->timestamp.time, last_message_time_)
      << DebugString() << " reading " << parts_message_reader_.filename();
  last_message_time_ = messages_.begin()->timestamp.time;
  return &(*messages_.begin());
}

void LogPartsSorter::PopFront() { messages_.erase(messages_.begin()); }

std::string LogPartsSorter::DebugString() const {
  std::stringstream ss;
  ss << "messages: [\n";
  int count = 0;
  bool no_dots = true;
  for (const Message &m : messages_) {
    if (count < 15 || count > static_cast<int>(messages_.size()) - 15) {
      ss << m << "\n";
    } else if (no_dots) {
      ss << "...\n";
      no_dots = false;
    }
    ++count;
  }
  ss << "] <- " << parts_message_reader_.filename();
  return ss.str();
}

NodeMerger::NodeMerger(std::vector<LogParts> parts) {
  CHECK_GE(parts.size(), 1u);
  // Enforce that we are sorting things only from a single node from a single
  // boot.
  const std::string_view part0_node = parts[0].node;
  const std::string_view part0_source_boot_uuid = parts[0].source_boot_uuid;
  for (size_t i = 1; i < parts.size(); ++i) {
    CHECK_EQ(part0_node, parts[i].node) << ": Can't merge different nodes.";
    CHECK_EQ(part0_source_boot_uuid, parts[i].source_boot_uuid)
        << ": Can't merge different boots.";
  }

  node_ = configuration::GetNodeIndex(parts[0].config.get(), part0_node);

  for (LogParts &part : parts) {
    parts_sorters_.emplace_back(std::move(part));
  }

  monotonic_start_time_ = monotonic_clock::max_time;
  realtime_start_time_ = realtime_clock::min_time;
  for (const LogPartsSorter &parts_sorter : parts_sorters_) {
    // We want to capture the earliest meaningful start time here. The start
    // time defaults to min_time when there's no meaningful value to report, so
    // let's ignore those.
    if (parts_sorter.monotonic_start_time() != monotonic_clock::min_time) {
      bool accept = false;
      // We want to prioritize start times from the logger node.  Really, we
      // want to prioritize start times with a valid realtime_clock time.  So,
      // if we have a start time without a RT clock, prefer a start time with a
      // RT clock, even it if is later.
      if (parts_sorter.realtime_start_time() != realtime_clock::min_time) {
        // We've got a good one.  See if the current start time has a good RT
        // clock, or if we should use this one instead.
        if (parts_sorter.monotonic_start_time() < monotonic_start_time_) {
          accept = true;
        } else if (realtime_start_time_ == realtime_clock::min_time) {
          // The previous start time doesn't have a good RT time, so it is very
          // likely the start time from a remote part file.  We just found a
          // better start time with a real RT time, so switch to that instead.
          accept = true;
        }
      } else if (realtime_start_time_ == realtime_clock::min_time) {
        // We don't have a RT time, so take the oldest.
        if (parts_sorter.monotonic_start_time() < monotonic_start_time_) {
          accept = true;
        }
      }

      if (accept) {
        monotonic_start_time_ = parts_sorter.monotonic_start_time();
        realtime_start_time_ = parts_sorter.realtime_start_time();
      }
    }
  }

  // If there was no meaningful start time reported, just use min_time.
  if (monotonic_start_time_ == monotonic_clock::max_time) {
    monotonic_start_time_ = monotonic_clock::min_time;
    realtime_start_time_ = realtime_clock::min_time;
  }
}

std::vector<const LogParts *> NodeMerger::Parts() const {
  std::vector<const LogParts *> p;
  p.reserve(parts_sorters_.size());
  for (const LogPartsSorter &parts_sorter : parts_sorters_) {
    p.emplace_back(&parts_sorter.parts());
  }
  return p;
}

Message *NodeMerger::Front() {
  // Return the current Front if we have one, otherwise go compute one.
  if (current_ != nullptr) {
    Message *result = current_->Front();
    CHECK_GE(result->timestamp.time, last_message_time_);
    return result;
  }

  // Otherwise, do a simple search for the oldest message, deduplicating any
  // duplicates.
  Message *oldest = nullptr;
  sorted_until_ = monotonic_clock::max_time;
  for (LogPartsSorter &parts_sorter : parts_sorters_) {
    Message *m = parts_sorter.Front();
    if (!m) {
      sorted_until_ = std::min(sorted_until_, parts_sorter.sorted_until());
      continue;
    }
    if (oldest == nullptr || *m < *oldest) {
      oldest = m;
      current_ = &parts_sorter;
    } else if (*m == *oldest) {
      // Found a duplicate.  If there is a choice, we want the one which has
      // the timestamp time.
      if (!m->data->has_monotonic_timestamp_time) {
        parts_sorter.PopFront();
      } else if (!oldest->data->has_monotonic_timestamp_time) {
        current_->PopFront();
        current_ = &parts_sorter;
        oldest = m;
      } else {
        CHECK_EQ(m->data->monotonic_timestamp_time,
                 oldest->data->monotonic_timestamp_time);
        parts_sorter.PopFront();
      }
    }

    // PopFront may change this, so compute it down here.
    sorted_until_ = std::min(sorted_until_, parts_sorter.sorted_until());
  }

  if (oldest) {
    CHECK_GE(oldest->timestamp.time, last_message_time_);
    last_message_time_ = oldest->timestamp.time;
    monotonic_oldest_time_ =
        std::min(monotonic_oldest_time_, oldest->timestamp.time);
  } else {
    last_message_time_ = monotonic_clock::max_time;
  }

  // Return the oldest message found.  This will be nullptr if nothing was
  // found, indicating there is nothing left.
  return oldest;
}

void NodeMerger::PopFront() {
  CHECK(current_ != nullptr) << "Popping before calling Front()";
  current_->PopFront();
  current_ = nullptr;
}

BootMerger::BootMerger(std::vector<LogParts> files) {
  std::vector<std::vector<LogParts>> boots;

  // Now, we need to split things out by boot.
  for (size_t i = 0; i < files.size(); ++i) {
    const size_t boot_count = files[i].boot_count;
    if (boot_count + 1 > boots.size()) {
      boots.resize(boot_count + 1);
    }
    boots[boot_count].emplace_back(std::move(files[i]));
  }

  node_mergers_.reserve(boots.size());
  for (size_t i = 0; i < boots.size(); ++i) {
    VLOG(2) << "Boot " << i;
    for (auto &p : boots[i]) {
      VLOG(2) << "Part " << p;
    }
    node_mergers_.emplace_back(
        std::make_unique<NodeMerger>(std::move(boots[i])));
  }
}

Message *BootMerger::Front() {
  Message *result = node_mergers_[index_]->Front();

  if (result != nullptr) {
    return result;
  }

  if (index_ + 1u == node_mergers_.size()) {
    // At the end of the last node merger, just return.
    return nullptr;
  } else {
    ++index_;
    return Front();
  }
}

void BootMerger::PopFront() { node_mergers_[index_]->PopFront(); }

std::vector<const LogParts *> BootMerger::Parts() const {
  std::vector<const LogParts *> results;
  for (const std::unique_ptr<NodeMerger> &node_merger : node_mergers_) {
    std::vector<const LogParts *> node_parts = node_merger->Parts();

    results.insert(results.end(), std::make_move_iterator(node_parts.begin()),
                   std::make_move_iterator(node_parts.end()));
  }

  return results;
}

TimestampMapper::TimestampMapper(std::vector<LogParts> parts)
    : boot_merger_(std::move(parts)),
      timestamp_callback_([](TimestampedMessage *) {}) {
  for (const LogParts *part : boot_merger_.Parts()) {
    if (!configuration_) {
      configuration_ = part->config;
    } else {
      CHECK_EQ(configuration_.get(), part->config.get());
    }
  }
  const Configuration *config = configuration_.get();
  // Only fill out nodes_data_ if there are nodes.  Otherwise everything gets
  // pretty simple.
  if (configuration::MultiNode(config)) {
    nodes_data_.resize(config->nodes()->size());
    const Node *my_node = config->nodes()->Get(node());
    for (size_t node_index = 0; node_index < nodes_data_.size(); ++node_index) {
      const Node *node = config->nodes()->Get(node_index);
      NodeData *node_data = &nodes_data_[node_index];
      node_data->channels.resize(config->channels()->size());
      // We should save the channel if it is delivered to the node represented
      // by the NodeData, but not sent by that node.  That combo means it is
      // forwarded.
      size_t channel_index = 0;
      node_data->any_delivered = false;
      for (const Channel *channel : *config->channels()) {
        node_data->channels[channel_index].delivered =
            configuration::ChannelIsReadableOnNode(channel, node) &&
            configuration::ChannelIsSendableOnNode(channel, my_node) &&
            (my_node != node);
        node_data->any_delivered = node_data->any_delivered ||
                                   node_data->channels[channel_index].delivered;
        if (node_data->channels[channel_index].delivered) {
          const Connection *connection =
              configuration::ConnectionToNode(channel, node);
          node_data->channels[channel_index].time_to_live =
              chrono::nanoseconds(connection->time_to_live());
        }
        ++channel_index;
      }
    }

    for (const Channel *channel : *config->channels()) {
      source_node_.emplace_back(configuration::GetNodeIndex(
          config, channel->source_node()->string_view()));
    }
  }
}

void TimestampMapper::AddPeer(TimestampMapper *timestamp_mapper) {
  CHECK(configuration::MultiNode(configuration()));
  CHECK_NE(timestamp_mapper->node(), node());
  CHECK_LT(timestamp_mapper->node(), nodes_data_.size());

  NodeData *node_data = &nodes_data_[timestamp_mapper->node()];
  // Only set it if this node delivers to the peer timestamp_mapper. Otherwise
  // we could needlessly save data.
  if (node_data->any_delivered) {
    VLOG(1) << "Registering on node " << node() << " for peer node "
            << timestamp_mapper->node();
    CHECK(timestamp_mapper->nodes_data_[node()].peer == nullptr);

    timestamp_mapper->nodes_data_[node()].peer = this;

    node_data->save_for_peer = true;
  }
}

void TimestampMapper::QueueMessage(Message *m) {
  matched_messages_.emplace_back(
      TimestampedMessage{.channel_index = m->channel_index,
                         .queue_index = m->queue_index,
                         .monotonic_event_time = m->timestamp,
                         .realtime_event_time = m->data->realtime_sent_time,
                         .remote_queue_index = BootQueueIndex::Invalid(),
                         .monotonic_remote_time = BootTimestamp::min_time(),
                         .realtime_remote_time = realtime_clock::min_time,
                         .monotonic_timestamp_time = BootTimestamp::min_time(),
                         .data = std::move(m->data)});
}

TimestampedMessage *TimestampMapper::Front() {
  // No need to fetch anything new.  A previous message still exists.
  switch (first_message_) {
    case FirstMessage::kNeedsUpdate:
      break;
    case FirstMessage::kInMessage:
      return &matched_messages_.front();
    case FirstMessage::kNullptr:
      return nullptr;
  }

  if (matched_messages_.empty()) {
    if (!QueueMatched()) {
      first_message_ = FirstMessage::kNullptr;
      return nullptr;
    }
  }
  first_message_ = FirstMessage::kInMessage;
  return &matched_messages_.front();
}

bool TimestampMapper::QueueMatched() {
  if (nodes_data_.empty()) {
    // Simple path.  We are single node, so there are no timestamps to match!
    CHECK_EQ(messages_.size(), 0u);
    Message *m = boot_merger_.Front();
    if (!m) {
      return false;
    }
    // Enqueue this message into matched_messages_ so we have a place to
    // associate remote timestamps, and return it.
    QueueMessage(m);

    CHECK_GE(matched_messages_.back().monotonic_event_time, last_message_time_);
    last_message_time_ = matched_messages_.back().monotonic_event_time;

    // We are thin wrapper around node_merger.  Call it directly.
    boot_merger_.PopFront();
    timestamp_callback_(&matched_messages_.back());
    return true;
  }

  // We need to only add messages to the list so they get processed for
  // messages which are delivered.  Reuse the flow below which uses messages_
  // by just adding the new message to messages_ and continuing.
  if (messages_.empty()) {
    if (!Queue()) {
      // Found nothing to add, we are out of data!
      return false;
    }

    // Now that it has been added (and cannibalized), forget about it
    // upstream.
    boot_merger_.PopFront();
  }

  Message *m = &(messages_.front());

  if (source_node_[m->channel_index] == node()) {
    // From us, just forward it on, filling the remote data in as invalid.
    QueueMessage(m);
    CHECK_GE(matched_messages_.back().monotonic_event_time, last_message_time_);
    last_message_time_ = matched_messages_.back().monotonic_event_time;
    messages_.pop_front();
    timestamp_callback_(&matched_messages_.back());
    return true;
  } else {
    // Got a timestamp, find the matching remote data, match it, and return
    // it.
    Message data = MatchingMessageFor(*m);

    // Return the data from the remote.  The local message only has timestamp
    // info which isn't relevant anymore once extracted.
    matched_messages_.emplace_back(TimestampedMessage{
        .channel_index = m->channel_index,
        .queue_index = m->queue_index,
        .monotonic_event_time = m->timestamp,
        .realtime_event_time = m->data->realtime_sent_time,
        .remote_queue_index =
            BootQueueIndex{.boot = m->monotonic_remote_boot,
                           .index = m->data->remote_queue_index.value()},
        .monotonic_remote_time = {m->monotonic_remote_boot,
                                  m->data->monotonic_remote_time.value()},
        .realtime_remote_time = m->data->realtime_remote_time.value(),
        .monotonic_timestamp_time = {m->monotonic_timestamp_boot,
                                     m->data->monotonic_timestamp_time},
        .data = std::move(data.data)});
    CHECK_GE(matched_messages_.back().monotonic_event_time, last_message_time_);
    last_message_time_ = matched_messages_.back().monotonic_event_time;
    // Since messages_ holds the data, drop it.
    messages_.pop_front();
    timestamp_callback_(&matched_messages_.back());
    return true;
  }
}

void TimestampMapper::QueueUntil(BootTimestamp queue_time) {
  while (last_message_time_ <= queue_time) {
    if (!QueueMatched()) {
      return;
    }
  }
}

void TimestampMapper::QueueFor(chrono::nanoseconds time_estimation_buffer) {
  // Note: queueing for time doesn't really work well across boots.  So we
  // just assume that if you are using this, you only care about the current
  // boot.
  //
  // TODO(austin): Is that the right concept?
  //
  // Make sure we have something queued first.  This makes the end time
  // calculation simpler, and is typically what folks want regardless.
  if (matched_messages_.empty()) {
    if (!QueueMatched()) {
      return;
    }
  }

  const aos::monotonic_clock::time_point end_queue_time =
      std::max(monotonic_start_time(
                   matched_messages_.front().monotonic_event_time.boot),
               matched_messages_.front().monotonic_event_time.time) +
      time_estimation_buffer;

  // Place sorted messages on the list until we have
  // --time_estimation_buffer_seconds seconds queued up (but queue at least
  // until the log starts).
  while (end_queue_time >= last_message_time_.time) {
    if (!QueueMatched()) {
      return;
    }
  }
}

void TimestampMapper::PopFront() {
  CHECK(first_message_ != FirstMessage::kNeedsUpdate);
  last_popped_message_time_ = Front()->monotonic_event_time;
  first_message_ = FirstMessage::kNeedsUpdate;

  matched_messages_.pop_front();
}

Message TimestampMapper::MatchingMessageFor(const Message &message) {
  // Figure out what queue index we are looking for.
  CHECK_NOTNULL(message.data);
  CHECK(message.data->remote_queue_index.has_value());
  const BootQueueIndex remote_queue_index =
      BootQueueIndex{.boot = message.monotonic_remote_boot,
                     .index = *message.data->remote_queue_index};

  CHECK(message.data->monotonic_remote_time.has_value());
  CHECK(message.data->realtime_remote_time.has_value());

  const BootTimestamp monotonic_remote_time{
      .boot = message.monotonic_remote_boot,
      .time = message.data->monotonic_remote_time.value()};
  const realtime_clock::time_point realtime_remote_time =
      *message.data->realtime_remote_time;

  TimestampMapper *peer =
      nodes_data_[source_node_[message.data->channel_index]].peer;

  // We only register the peers which we have data for.  So, if we are being
  // asked to pull a timestamp from a peer which doesn't exist, return an
  // empty message.
  if (peer == nullptr) {
    // TODO(austin): Make sure the tests hit all these paths with a boot count
    // of 1...
    return Message{.channel_index = message.channel_index,
                   .queue_index = remote_queue_index,
                   .timestamp = monotonic_remote_time,
                   .monotonic_remote_boot = 0xffffff,
                   .monotonic_timestamp_boot = 0xffffff,
                   .data = nullptr};
  }

  // The queue which will have the matching data, if available.
  std::deque<Message> *data_queue =
      &peer->nodes_data_[node()].channels[message.channel_index].messages;

  peer->QueueUnmatchedUntil(monotonic_remote_time);

  if (data_queue->empty()) {
    return Message{.channel_index = message.channel_index,
                   .queue_index = remote_queue_index,
                   .timestamp = monotonic_remote_time,
                   .monotonic_remote_boot = 0xffffff,
                   .monotonic_timestamp_boot = 0xffffff,
                   .data = nullptr};
  }

  if (remote_queue_index < data_queue->front().queue_index ||
      remote_queue_index > data_queue->back().queue_index) {
    return Message{.channel_index = message.channel_index,
                   .queue_index = remote_queue_index,
                   .timestamp = monotonic_remote_time,
                   .monotonic_remote_boot = 0xffffff,
                   .monotonic_timestamp_boot = 0xffffff,
                   .data = nullptr};
  }

  // The algorithm below is constant time with some assumptions.  We need there
  // to be no missing messages in the data stream.  This also assumes a queue
  // hasn't wrapped.  That is conservative, but should let us get started.
  if (data_queue->back().queue_index.boot ==
          data_queue->front().queue_index.boot &&
      (data_queue->back().queue_index.index -
           data_queue->front().queue_index.index + 1u ==
       data_queue->size())) {
    CHECK_EQ(remote_queue_index.boot, data_queue->front().queue_index.boot);
    // Pull the data out and confirm that the timestamps match as expected.
    //
    // TODO(austin): Move if not reliable.
    Message result = (*data_queue)[remote_queue_index.index -
                                   data_queue->front().queue_index.index];

    CHECK_EQ(result.timestamp, monotonic_remote_time)
        << ": Queue index matches, but timestamp doesn't.  Please investigate!";
    CHECK_EQ(result.data->realtime_sent_time, realtime_remote_time)
        << ": Queue index matches, but timestamp doesn't.  Please investigate!";
    // Now drop the data off the front.  We have deduplicated timestamps, so we
    // are done.  And all the data is in order.
    data_queue->erase(
        data_queue->begin(),
        data_queue->begin() +
            (remote_queue_index.index - data_queue->front().queue_index.index));
    return result;
  } else {
    // TODO(austin): Binary search.
    auto it = std::find_if(
        data_queue->begin(), data_queue->end(),
        [remote_queue_index,
         remote_boot = monotonic_remote_time.boot](const Message &m) {
          return m.queue_index == remote_queue_index &&
                 m.timestamp.boot == remote_boot;
        });
    if (it == data_queue->end()) {
      return Message{.channel_index = message.channel_index,
                     .queue_index = remote_queue_index,
                     .timestamp = monotonic_remote_time,
                     .monotonic_remote_boot = 0xffffff,
                     .monotonic_timestamp_boot = 0xffffff,
                     .data = nullptr};
    }

    Message result = std::move(*it);

    CHECK_EQ(result.timestamp, monotonic_remote_time)
        << ": Queue index matches, but timestamp doesn't.  Please "
           "investigate!";
    CHECK_EQ(result.data->realtime_sent_time, realtime_remote_time)
        << ": Queue index matches, but timestamp doesn't.  Please "
           "investigate!";

    // Erase everything up to this message.  We want to keep 1 message in the
    // queue so we can handle reliable messages forwarded across boots.
    data_queue->erase(data_queue->begin(), it);

    return result;
  }
}

void TimestampMapper::QueueUnmatchedUntil(BootTimestamp t) {
  if (queued_until_ > t) {
    return;
  }
  while (true) {
    if (!messages_.empty() && messages_.back().timestamp > t) {
      queued_until_ = std::max(queued_until_, messages_.back().timestamp);
      return;
    }

    if (!Queue()) {
      // Found nothing to add, we are out of data!
      queued_until_ = BootTimestamp::max_time();
      return;
    }

    // Now that it has been added (and cannibalized), forget about it
    // upstream.
    boot_merger_.PopFront();
  }
}

bool TimestampMapper::Queue() {
  Message *m = boot_merger_.Front();
  if (m == nullptr) {
    return false;
  }
  for (NodeData &node_data : nodes_data_) {
    if (!node_data.any_delivered) continue;
    if (!node_data.save_for_peer) continue;
    if (node_data.channels[m->channel_index].delivered) {
      // If we have data but no timestamps (logs where the timestamps didn't get
      // logged are classic), we can grow this indefinitely.  We don't need to
      // keep anything that is older than the last message returned.

      // We have the time on the source node.
      // We care to wait until we have the time on the destination node.
      std::deque<Message> &messages =
          node_data.channels[m->channel_index].messages;
      // Max delay over the network is the TTL, so let's take the queue time and
      // add TTL to it.  Don't forget any messages which are reliable until
      // someone can come up with a good reason to forget those too.
      if (node_data.channels[m->channel_index].time_to_live >
          chrono::nanoseconds(0)) {
        // We need to make *some* assumptions about network delay for this to
        // work.  We want to only look at the RX side.  This means we need to
        // track the last time a message was popped from any channel from the
        // node sending this message, and compare that to the max time we expect
        // that a message will take to be delivered across the network.  This
        // assumes that messages are popped in time order as a proxy for
        // measuring the distributed time at this layer.
        //
        // Leave at least 1 message in here so we can handle reboots and
        // messages getting sent twice.
        while (messages.size() > 1u &&
               messages.begin()->timestamp +
                       node_data.channels[m->channel_index].time_to_live +
                       chrono::duration_cast<chrono::nanoseconds>(
                           chrono::duration<double>(FLAGS_max_network_delay)) <
                   last_popped_message_time_) {
          messages.pop_front();
        }
      }
      node_data.channels[m->channel_index].messages.emplace_back(*m);
    }
  }

  messages_.emplace_back(std::move(*m));
  return true;
}

std::string TimestampMapper::DebugString() const {
  std::stringstream ss;
  ss << "node " << node() << " (" << node_name() << ") [\n";
  for (const Message &message : messages_) {
    ss << "  " << message << "\n";
  }
  ss << "] queued_until " << queued_until_;
  for (const NodeData &ns : nodes_data_) {
    if (ns.peer == nullptr) continue;
    ss << "\nnode " << ns.peer->node() << " remote_data [\n";
    size_t channel_index = 0;
    for (const NodeData::ChannelData &channel_data :
         ns.peer->nodes_data_[node()].channels) {
      if (channel_data.messages.empty()) {
        continue;
      }

      ss << "  channel " << channel_index << " [\n";
      for (const Message &m : channel_data.messages) {
        ss << "    " << m << "\n";
      }
      ss << "  ]\n";
      ++channel_index;
    }
    ss << "] queued_until " << ns.peer->queued_until_;
  }
  return ss.str();
}

std::string MaybeNodeName(const Node *node) {
  if (node != nullptr) {
    return node->name()->str() + " ";
  }
  return "";
}

}  // namespace aos::logger
