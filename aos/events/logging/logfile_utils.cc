#include "aos/events/logging/logfile_utils.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>

#include <algorithm>
#include <climits>

#include "absl/strings/escaping.h"
#include "aos/configuration.h"
#include "aos/flatbuffer_merge.h"
#include "aos/util/file.h"
#include "flatbuffers/flatbuffers.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#if defined(__x86_64__)
#define ENABLE_LZMA 1
#elif defined(__aarch64__)
#define ENABLE_LZMA 1
#else
#define ENABLE_LZMA 0
#endif

#if ENABLE_LZMA
#include "aos/events/logging/lzma_encoder.h"
#endif

DEFINE_int32(flush_size, 128000,
             "Number of outstanding bytes to allow before flushing to disk.");

namespace aos::logger {

namespace chrono = std::chrono;

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
      PCHECK(fd_ != -1) << ": Failed to open " << filename << " for writing";
      VLOG(1) << "Opened " << filename << " for writing";
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
  } else {
    encoder_->Encode(CopySpanAsDetachedBuffer(span));
  }

  FlushAtThreshold();
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
  VLOG(1) << "Closed " << filename_;
}

void DetachedBufferWriter::Flush() {
  const auto queue = encoder_->queue();
  if (queue.empty()) {
    return;
  }
  if (ran_out_of_space_) {
    // We don't want any later data to be written after space becomes available,
    // so refuse to write anything more once we've dropped data because we ran
    // out of space.
    VLOG(1) << "Ignoring queue: " << queue.size();
    encoder_->Clear(queue.size());
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

void DetachedBufferWriter::FlushAtThreshold() {
  // Flush if we are at the max number of iovs per writev, because there's no
  // point queueing up any more data in memory. Also flush once we have enough
  // data queued up.
  while (encoder_->queued_bytes() > static_cast<size_t>(FLAGS_flush_size) ||
         encoder_->queue_size() >= IOV_MAX) {
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
  static const std::string_view kXz = ".xz";
  if (filename.substr(filename.size() - kXz.size()) == kXz) {
#if ENABLE_LZMA
    decoder_ = std::make_unique<LzmaDecoder>(filename);
#else
    LOG(FATAL) << "Reading xz-compressed files not supported on this platform";
#endif
  } else {
    decoder_ = std::make_unique<DummyDecoder>(filename);
  }
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
    std::string_view filename) {
  SpanReader span_reader(filename);
  absl::Span<const uint8_t> config_data = span_reader.ReadMessage();

  // Make sure something was read.
  if (config_data == absl::Span<const uint8_t>()) {
    return std::nullopt;
  }

  // And copy the config so we have it forever, removing the size prefix.
  ResizeableBuffer data;
  data.resize(config_data.size());
  memcpy(data.data(), config_data.begin(), data.size());
  SizePrefixedFlatbufferVector<LogFileHeader> result(std::move(data));
  if (!result.Verify()) {
    return std::nullopt;
  }
  return result;
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
  ResizeableBuffer data;
  data.resize(data_span.size());
  memcpy(data.data(), data_span.begin(), data.size());
  SizePrefixedFlatbufferVector<MessageHeader> result(std::move(data));
  if (!result.Verify()) {
    return std::nullopt;
  }
  return result;
}

MessageReader::MessageReader(std::string_view filename)
    : span_reader_(filename),
      raw_log_file_header_(
          SizePrefixedFlatbufferVector<LogFileHeader>::Empty()) {
  // Make sure we have enough to read the size.
  absl::Span<const uint8_t> header_data = span_reader_.ReadMessage();

  // Make sure something was read.
  CHECK(header_data != absl::Span<const uint8_t>())
      << ": Failed to read header from: " << filename;

  // And copy the header data so we have it forever.
  ResizeableBuffer header_data_copy;
  header_data_copy.resize(header_data.size());
  memcpy(header_data_copy.data(), header_data.begin(), header_data_copy.size());
  raw_log_file_header_ =
      SizePrefixedFlatbufferVector<LogFileHeader>(std::move(header_data_copy));

  max_out_of_order_duration_ =
      chrono::nanoseconds(log_file_header()->max_out_of_order_duration());

  VLOG(1) << "Opened " << filename << " as node "
          << FlatbufferToJson(log_file_header()->node());
}

std::optional<SizePrefixedFlatbufferVector<MessageHeader>>
MessageReader::ReadMessage() {
  absl::Span<const uint8_t> msg_data = span_reader_.ReadMessage();
  if (msg_data == absl::Span<const uint8_t>()) {
    return std::nullopt;
  }

  ResizeableBuffer result_buffer;
  result_buffer.resize(msg_data.size());
  memcpy(result_buffer.data(), msg_data.begin(), result_buffer.size());
  SizePrefixedFlatbufferVector<MessageHeader> result(std::move(result_buffer));

  const monotonic_clock::time_point timestamp = monotonic_clock::time_point(
      chrono::nanoseconds(result.message().monotonic_sent_time()));

  newest_timestamp_ = std::max(newest_timestamp_, timestamp);
  VLOG(2) << "Read from " << filename() << " data " << FlatbufferToJson(result);
  return std::move(result);
}

PartsMessageReader::PartsMessageReader(LogParts log_parts)
    : parts_(std::move(log_parts)), message_reader_(parts_.parts[0]) {}

std::optional<SizePrefixedFlatbufferVector<MessageHeader>>
PartsMessageReader::ReadMessage() {
  while (!done_) {
    std::optional<SizePrefixedFlatbufferVector<MessageHeader>> message =
        message_reader_.ReadMessage();
    if (message) {
      newest_timestamp_ = message_reader_.newest_timestamp();
      const monotonic_clock::time_point monotonic_sent_time(
          chrono::nanoseconds(message->message().monotonic_sent_time()));
      // TODO(austin): Does this work with startup?  Might need to use the start
      // time.
      // TODO(austin): Does this work with startup when we don't know the remote
      // start time too?  Look at one of those logs to compare.
      if (monotonic_sent_time >
          parts_.monotonic_start_time + max_out_of_order_duration()) {
        after_start_ = true;
      }
      if (after_start_) {
        CHECK_GE(monotonic_sent_time,
                 newest_timestamp_ - max_out_of_order_duration())
            << ": Max out of order exceeded. " << parts_ << ", start time is "
            << parts_.monotonic_start_time << " currently reading "
            << filename();
      }
      return message;
    }
    NextLog();
  }
  newest_timestamp_ = monotonic_clock::max_time;
  return std::nullopt;
}

void PartsMessageReader::NextLog() {
  if (next_part_index_ == parts_.parts.size()) {
    done_ = true;
    return;
  }
  message_reader_ = MessageReader(parts_.parts[next_part_index_]);
  ++next_part_index_;
}

bool Message::operator<(const Message &m2) const {
  if (this->timestamp < m2.timestamp) {
    return true;
  } else if (this->timestamp > m2.timestamp) {
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
  return timestamp == m2.timestamp && channel_index == m2.channel_index &&
         queue_index == m2.queue_index;
}

std::ostream &operator<<(std::ostream &os, const Message &m) {
  os << "{.channel_index=" << m.channel_index
     << ", .queue_index=" << m.queue_index << ", .timestamp=" << m.timestamp;
  if (m.data.Verify()) {
    os << ", .data="
       << aos::FlatbufferToJson(m.data,
                                {.multi_line = false, .max_vector_size = 1});
  }
  os << "}";
  return os;
}

std::ostream &operator<<(std::ostream &os, const TimestampedMessage &m) {
  os << "{.channel_index=" << m.channel_index
     << ", .queue_index=" << m.queue_index
     << ", .monotonic_event_time=" << m.monotonic_event_time
     << ", .realtime_event_time=" << m.realtime_event_time;
  if (m.remote_queue_index != 0xffffffff) {
    os << ", .remote_queue_index=" << m.remote_queue_index;
  }
  if (m.monotonic_remote_time != monotonic_clock::min_time) {
    os << ", .monotonic_remote_time=" << m.monotonic_remote_time;
  }
  if (m.realtime_remote_time != realtime_clock::min_time) {
    os << ", .realtime_remote_time=" << m.realtime_remote_time;
  }
  if (m.data.Verify()) {
    os << ", .data="
       << aos::FlatbufferToJson(m.data,
                                {.multi_line = false, .max_vector_size = 1});
  }
  os << "}";
  return os;
}

LogPartsSorter::LogPartsSorter(LogParts log_parts)
    : parts_message_reader_(log_parts) {}

Message *LogPartsSorter::Front() {
  // Queue up data until enough data has been queued that the front message is
  // sorted enough to be safe to pop.  This may do nothing, so we should make
  // sure the nothing path is checked quickly.
  if (sorted_until() != monotonic_clock::max_time) {
    while (true) {
      if (!messages_.empty() && messages_.begin()->timestamp < sorted_until() &&
          sorted_until() >= monotonic_start_time()) {
        break;
      }

      std::optional<SizePrefixedFlatbufferVector<MessageHeader>> m =
          parts_message_reader_.ReadMessage();
      // No data left, sorted forever, work through what is left.
      if (!m) {
        sorted_until_ = monotonic_clock::max_time;
        break;
      }

      messages_.insert(
          {.channel_index = m.value().message().channel_index(),
           .queue_index = m.value().message().queue_index(),
           .timestamp = monotonic_clock::time_point(std::chrono::nanoseconds(
               m.value().message().monotonic_sent_time())),
           .data = std::move(m.value())});

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

  CHECK_GE(messages_.begin()->timestamp, last_message_time_)
      << DebugString() << " reading " << parts_message_reader_.filename();
  last_message_time_ = messages_.begin()->timestamp;
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
  const std::string part0_node = parts[0].node;
  for (size_t i = 1; i < parts.size(); ++i) {
    CHECK_EQ(part0_node, parts[i].node) << ": Can't merge different nodes.";
  }
  for (LogParts &part : parts) {
    parts_sorters_.emplace_back(std::move(part));
  }

  node_ = configuration::GetNodeIndex(configuration(), part0_node);

  monotonic_start_time_ = monotonic_clock::max_time;
  realtime_start_time_ = realtime_clock::max_time;
  for (const LogPartsSorter &parts_sorter : parts_sorters_) {
    if (parts_sorter.monotonic_start_time() < monotonic_start_time_) {
      monotonic_start_time_ = parts_sorter.monotonic_start_time();
      realtime_start_time_ = parts_sorter.realtime_start_time();
    }
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
    CHECK_GE(result->timestamp, last_message_time_);
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
      // Found a duplicate.  It doesn't matter which one we return.  It is
      // easiest to just drop the new one.
      parts_sorter.PopFront();
    }

    // PopFront may change this, so compute it down here.
    sorted_until_ = std::min(sorted_until_, parts_sorter.sorted_until());
  }

  if (oldest) {
    CHECK_GE(oldest->timestamp, last_message_time_);
    last_message_time_ = oldest->timestamp;
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

TimestampMapper::TimestampMapper(std::vector<LogParts> parts)
    : node_merger_(std::move(parts)),
      message_{.channel_index = 0xffffffff,
               .queue_index = 0xffffffff,
               .monotonic_event_time = monotonic_clock::min_time,
               .realtime_event_time = realtime_clock::min_time,
               .remote_queue_index = 0xffffffff,
               .monotonic_remote_time = monotonic_clock::min_time,
               .realtime_remote_time = realtime_clock::min_time,
               .data = SizePrefixedFlatbufferVector<MessageHeader>::Empty()} {
  for (const LogParts *part : node_merger_.Parts()) {
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
            configuration::ChannelIsSendableOnNode(channel, my_node);
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
  // Only set it if this node delivers to the peer timestamp_mapper.  Otherwise
  // we could needlessly save data.
  if (node_data->any_delivered) {
    LOG(INFO) << "Registering on node " << node() << " for peer node "
              << timestamp_mapper->node();
    CHECK(timestamp_mapper->nodes_data_[node()].peer == nullptr);

    timestamp_mapper->nodes_data_[node()].peer = this;
  }
}

void TimestampMapper::FillMessage(Message *m) {
  message_ = {
      .channel_index = m->channel_index,
      .queue_index = m->queue_index,
      .monotonic_event_time = m->timestamp,
      .realtime_event_time = aos::realtime_clock::time_point(
          std::chrono::nanoseconds(m->data.message().realtime_sent_time())),
      .remote_queue_index = 0xffffffff,
      .monotonic_remote_time = monotonic_clock::min_time,
      .realtime_remote_time = realtime_clock::min_time,
      .data = std::move(m->data)};
}

TimestampedMessage *TimestampMapper::Front() {
  // No need to fetch anything new.  A previous message still exists.
  switch (first_message_) {
    case FirstMessage::kNeedsUpdate:
      break;
    case FirstMessage::kInMessage:
      return &message_;
    case FirstMessage::kNullptr:
      return nullptr;
  }

  if (nodes_data_.empty()) {
    // Simple path.  We are single node, so there are no timestamps to match!
    CHECK_EQ(messages_.size(), 0u);
    Message *m = node_merger_.Front();
    if (!m) {
      first_message_ = FirstMessage::kNullptr;
      return nullptr;
    }
    // Fill in message_ so we have a place to associate remote timestamps, and
    // return it.
    FillMessage(m);

    CHECK_GE(message_.monotonic_event_time, last_message_time_);
    last_message_time_ = message_.monotonic_event_time;
    first_message_ = FirstMessage::kInMessage;
    return &message_;
  }

  // We need to only add messages to the list so they get processed for messages
  // which are delivered.  Reuse the flow below which uses messages_ by just
  // adding the new message to messages_ and continuing.
  if (messages_.empty()) {
    if (!Queue()) {
      // Found nothing to add, we are out of data!
      first_message_ = FirstMessage::kNullptr;
      return nullptr;
    }

    // Now that it has been added (and cannibalized), forget about it upstream.
    node_merger_.PopFront();
  }

  Message *m = &(messages_.front());

  if (source_node_[m->channel_index] == node()) {
    // From us, just forward it on, filling the remote data in as invalid.
    FillMessage(m);
    CHECK_GE(message_.monotonic_event_time, last_message_time_);
    last_message_time_ = message_.monotonic_event_time;
    first_message_ = FirstMessage::kInMessage;
    return &message_;
  } else {
    // Got a timestamp, find the matching remote data, match it, and return it.
    Message data = MatchingMessageFor(*m);

    // Return the data from the remote.  The local message only has timestamp
    // info which isn't relevant anymore once extracted.
    message_ = {
        .channel_index = m->channel_index,
        .queue_index = m->queue_index,
        .monotonic_event_time = m->timestamp,
        .realtime_event_time = aos::realtime_clock::time_point(
            std::chrono::nanoseconds(m->data.message().realtime_sent_time())),
        .remote_queue_index = m->data.message().remote_queue_index(),
        .monotonic_remote_time =
            monotonic_clock::time_point(std::chrono::nanoseconds(
                m->data.message().monotonic_remote_time())),
        .realtime_remote_time = realtime_clock::time_point(
            std::chrono::nanoseconds(m->data.message().realtime_remote_time())),
        .data = std::move(data.data)};
    CHECK_GE(message_.monotonic_event_time, last_message_time_);
    last_message_time_ = message_.monotonic_event_time;
    first_message_ = FirstMessage::kInMessage;
    return &message_;
  }
}

void TimestampMapper::PopFront() {
  CHECK(first_message_ != FirstMessage::kNeedsUpdate);
  first_message_ = FirstMessage::kNeedsUpdate;

  if (nodes_data_.empty()) {
    // We are thin wrapper around node_merger.  Call it directly.
    node_merger_.PopFront();
  } else {
    // Since messages_ holds the data, drop it.
    messages_.pop_front();
  }
}

Message TimestampMapper::MatchingMessageFor(const Message &message) {
  // Figure out what queue index we are looking for.
  CHECK(message.data.message().has_remote_queue_index());
  const uint32_t remote_queue_index =
      message.data.message().remote_queue_index();

  CHECK(message.data.message().has_monotonic_remote_time());
  CHECK(message.data.message().has_realtime_remote_time());

  const monotonic_clock::time_point monotonic_remote_time(
      std::chrono::nanoseconds(message.data.message().monotonic_remote_time()));
  const realtime_clock::time_point realtime_remote_time(
      std::chrono::nanoseconds(message.data.message().realtime_remote_time()));

  TimestampMapper *peer = nodes_data_[source_node_[message.channel_index]].peer;

  // We only register the peers which we have data for.  So, if we are being
  // asked to pull a timestamp from a peer which doesn't exist, return an empty
  // message.
  if (peer == nullptr) {
    return Message{
        .channel_index = message.channel_index,
        .queue_index = remote_queue_index,
        .timestamp = monotonic_remote_time,
        .data = SizePrefixedFlatbufferVector<MessageHeader>::Empty()};
  }

  // The queue which will have the matching data, if available.
  std::deque<Message> *data_queue =
      &peer->nodes_data_[node()].channels[message.channel_index].messages;

  peer->QueueUntil(monotonic_remote_time);

  if (data_queue->empty()) {
    return Message{
        .channel_index = message.channel_index,
        .queue_index = remote_queue_index,
        .timestamp = monotonic_remote_time,
        .data = SizePrefixedFlatbufferVector<MessageHeader>::Empty()};
  }

  if (remote_queue_index < data_queue->front().queue_index ||
      remote_queue_index > data_queue->back().queue_index) {
    return Message{
        .channel_index = message.channel_index,
        .queue_index = remote_queue_index,
        .timestamp = monotonic_remote_time,
        .data = SizePrefixedFlatbufferVector<MessageHeader>::Empty()};
  }

  // The algorithm below is constant time with some assumptions.  We need there
  // to be no missing messages in the data stream.  This also assumes a queue
  // hasn't wrapped.  That is conservative, but should let us get started.
  if (data_queue->back().queue_index - data_queue->front().queue_index + 1u ==
      data_queue->size()) {
    // Pull the data out and confirm that the timestamps match as expected.
    Message result = std::move(
        (*data_queue)[remote_queue_index - data_queue->front().queue_index]);

    CHECK_EQ(result.timestamp, monotonic_remote_time)
        << ": Queue index matches, but timestamp doesn't.  Please investigate!";
    CHECK_EQ(realtime_clock::time_point(std::chrono::nanoseconds(
                 result.data.message().realtime_sent_time())),
             realtime_remote_time)
        << ": Queue index matches, but timestamp doesn't.  Please investigate!";
    // Now drop the data off the front.  We have deduplicated timestamps, so we
    // are done.  And all the data is in order.
    data_queue->erase(data_queue->begin(),
                      data_queue->begin() + (1 + remote_queue_index -
                                             data_queue->front().queue_index));
    return result;
  } else {
    auto it = std::find_if(data_queue->begin(), data_queue->end(),
                           [remote_queue_index](const Message &m) {
                             return m.queue_index == remote_queue_index;
                           });
    if (it == data_queue->end()) {
      return Message{
          .channel_index = message.channel_index,
          .queue_index = remote_queue_index,
          .timestamp = monotonic_remote_time,
          .data = SizePrefixedFlatbufferVector<MessageHeader>::Empty()};
    }

    Message result = std::move(*it);

    CHECK_EQ(result.timestamp, monotonic_remote_time)
        << ": Queue index matches, but timestamp doesn't.  Please investigate!";
    CHECK_EQ(realtime_clock::time_point(std::chrono::nanoseconds(
                 result.data.message().realtime_sent_time())),
             realtime_remote_time)
        << ": Queue index matches, but timestamp doesn't.  Please investigate!";

    data_queue->erase(it);

    return result;
  }
}

void TimestampMapper::QueueUntil(monotonic_clock::time_point t) {
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
      queued_until_ = monotonic_clock::max_time;
      return;
    }

    // Now that it has been added (and cannibalized), forget about it upstream.
    node_merger_.PopFront();
  }
}

bool TimestampMapper::Queue() {
  Message *m = node_merger_.Front();
  if (m == nullptr) {
    return false;
  }
  for (NodeData &node_data : nodes_data_) {
    if (!node_data.any_delivered) continue;
    if (node_data.channels[m->channel_index].delivered) {
      // TODO(austin): This copies the data...  Probably not worth stressing
      // about yet.
      // TODO(austin): Bound how big this can get.  We tend not to send massive
      // data, so we can probably ignore this for a bit.
      node_data.channels[m->channel_index].messages.emplace_back(*m);
    }
  }

  messages_.emplace_back(std::move(*m));
  return true;
}

std::string TimestampMapper::DebugString() const {
  std::stringstream ss;
  ss << "node " << node() << " [\n";
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
