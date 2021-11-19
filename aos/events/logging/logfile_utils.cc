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
    max_out_of_order, -1,
    "If set, this overrides the max out of order duration for a log file.");

DEFINE_bool(workaround_double_headers, true,
            "Some old log files have two headers at the beginning.  Use the "
            "last header as the actual header.");

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

flatbuffers::Offset<MessageHeader> PackMessage(
    flatbuffers::FlatBufferBuilder *fbb, const Context &context,
    int channel_index, LogType log_type) {
  flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset;

  switch (log_type) {
    case LogType::kLogMessage:
    case LogType::kLogMessageAndDeliveryTime:
    case LogType::kLogRemoteMessage:
      data_offset = fbb->CreateVector(
          static_cast<const uint8_t *>(context.data), context.size);
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

SpanReader::SpanReader(std::string_view filename) : filename_(filename) {
  decoder_ = std::make_unique<DummyDecoder>(filename);

  static constexpr std::string_view kXz = ".xz";
  static constexpr std::string_view kSnappy = SnappyDecoder::kExtension;
  if (filename.substr(filename.size() - kXz.size()) == kXz) {
#if ENABLE_LZMA
    decoder_ = std::make_unique<ThreadedLzmaDecoder>(std::move(decoder_));
#else
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
  consumed_data_ +=
      flatbuffers::GetPrefixedSize(data_.data() + consumed_data_) +
      sizeof(flatbuffers::uoffset_t);
}

absl::Span<const uint8_t> SpanReader::ReadMessage() {
  absl::Span<const uint8_t> result = PeekMessage();
  if (result != absl::Span<const uint8_t>()) {
    ConsumeMessage();
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
  std::optional<SizePrefixedFlatbufferVector<LogFileHeader>>
      raw_log_file_header = ReadHeader(&span_reader_);

  // Make sure something was read.
  CHECK(raw_log_file_header) << ": Failed to read header from: " << filename;

  raw_log_file_header_ = std::move(*raw_log_file_header);

  CHECK(raw_log_file_header_.Verify()) << "Log file header is corrupted";

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
    return nullptr;
  }

  SizePrefixedFlatbufferSpan<MessageHeader> msg(msg_data);
  CHECK(msg.Verify()) << ": Corrupted message from " << filename();

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
        const Node *node = parts().config->nodes()->Get(
            source_node_index_[m->channel_index]);

        std::optional<size_t> boot = parts_message_reader_.boot_count(
            source_node_index_[m->channel_index]);
        CHECK(boot) << ": Failed to find boot for node " << MaybeNodeName(node)
                    << ", with index "
                    << source_node_index_[m->channel_index];
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
  matched_messages_.emplace_back(TimestampedMessage{
      .channel_index = m->channel_index,
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
    return Message{
        .channel_index = message.channel_index,
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
    CHECK_EQ(result.data->realtime_sent_time,
             realtime_remote_time)
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

    // TODO(austin): We still go in order, so we can erase from the beginning to
    // our iterator minus 1.  That'll keep 1 in the queue.
    data_queue->erase(it);

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
      // TODO(austin): This copies the data...  Probably not worth stressing
      // about yet.
      // TODO(austin): Bound how big this can get.  We tend not to send
      // massive data, so we can probably ignore this for a bit.
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
