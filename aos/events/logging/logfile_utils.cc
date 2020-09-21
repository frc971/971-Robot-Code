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

DEFINE_int32(flush_size, 128000,
             "Number of outstanding bytes to allow before flushing to disk.");

namespace aos::logger {

namespace chrono = std::chrono;

DetachedBufferWriter::DetachedBufferWriter(
    std::string_view filename, std::unique_ptr<DetachedBufferEncoder> encoder)
    : filename_(filename), encoder_(std::move(encoder)) {
  util::MkdirP(filename, 0777);
  fd_ = open(std::string(filename).c_str(),
             O_RDWR | O_CLOEXEC | O_CREAT | O_EXCL, 0774);
  VLOG(1) << "Opened " << filename << " for writing";
  PCHECK(fd_ != -1) << ": Failed to open " << filename << " for writing";
}

DetachedBufferWriter::~DetachedBufferWriter() {
  encoder_->Finish();
  while (encoder_->queue_size() > 0) {
    Flush();
  }
  PLOG_IF(ERROR, close(fd_) == -1) << " Failed to close logfile";
  VLOG(1) << "Closed " << filename_;
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
    PCHECK(written >= 0) << ": write failed";
    CHECK_EQ(written, static_cast<ssize_t>(span.size()))
        << ": Wrote " << written << " expected " << span.size();
    UpdateStatsForWrite(end - start, written, 1);
  } else {
    encoder_->Encode(CopySpanAsDetachedBuffer(span));
  }

  FlushAtThreshold();
}

void DetachedBufferWriter::Flush() {
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
  PCHECK(written >= 0) << ": writev failed";
  // TODO(austin): Handle partial writes in some way other than crashing...
  CHECK_EQ(written, static_cast<ssize_t>(counted_size))
      << ": Wrote " << written << " expected " << counted_size;

  encoder_->Clear(iovec_size);

  UpdateStatsForWrite(end - start, written, iovec_size);
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
  // Support for other kinds of decoders based on the filename should be added
  // here.
  decoder_ = std::make_unique<DummyDecoder>(filename);
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

FlatbufferVector<LogFileHeader> ReadHeader(std::string_view filename) {
  SpanReader span_reader(filename);
  absl::Span<const uint8_t> config_data = span_reader.ReadMessage();

  // Make sure something was read.
  CHECK(config_data != absl::Span<const uint8_t>())
      << ": Failed to read header from: " << filename;

  // And copy the config so we have it forever, removing the size prefix.
  std::vector<uint8_t> data(
      config_data.begin() + sizeof(flatbuffers::uoffset_t), config_data.end());
  return FlatbufferVector<LogFileHeader>(std::move(data));
}

FlatbufferVector<MessageHeader> ReadNthMessage(std::string_view filename,
                                               size_t n) {
  SpanReader span_reader(filename);
  absl::Span<const uint8_t> data_span = span_reader.ReadMessage();
  for (size_t i = 0; i < n + 1; ++i) {
    data_span = span_reader.ReadMessage();

    // Make sure something was read.
    CHECK(data_span != absl::Span<const uint8_t>())
        << ": Failed to read data from: " << filename;
  }

  // And copy the data so we have it forever.
  std::vector<uint8_t> data(data_span.begin() + sizeof(flatbuffers::uoffset_t),
                            data_span.end());
  return FlatbufferVector<MessageHeader>(std::move(data));
}

MessageReader::MessageReader(std::string_view filename)
    : span_reader_(filename),
      raw_log_file_header_(FlatbufferVector<LogFileHeader>::Empty()) {
  // Make sure we have enough to read the size.
  absl::Span<const uint8_t> header_data = span_reader_.ReadMessage();

  // Make sure something was read.
  CHECK(header_data != absl::Span<const uint8_t>())
      << ": Failed to read header from: " << filename;

  // And copy the header data so we have it forever.
  std::vector<uint8_t> header_data_copy(
      header_data.begin() + sizeof(flatbuffers::uoffset_t), header_data.end());
  raw_log_file_header_ =
      FlatbufferVector<LogFileHeader>(std::move(header_data_copy));

  max_out_of_order_duration_ =
      chrono::nanoseconds(log_file_header()->max_out_of_order_duration());

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
      log_file_header_(FlatbufferVector<LogFileHeader>::Empty()) {
  CHECK(NextLogFile()) << ": filenames is empty.  Need files to read.";

  // Grab any log file header.  They should all match (and we will check as we
  // open more of them).
  log_file_header_ = message_reader_->raw_log_file_header();

  for (size_t i = 1; i < filenames_.size(); ++i) {
    MessageReader message_reader(filenames_[i]);

    const monotonic_clock::time_point new_monotonic_start_time(
        chrono::nanoseconds(
            message_reader.log_file_header()->monotonic_start_time()));
    const realtime_clock::time_point new_realtime_start_time(
        chrono::nanoseconds(
            message_reader.log_file_header()->realtime_start_time()));

    // There are 2 types of part files.  Part files from before time estimation
    // has started, and part files after.  We don't declare a log file "started"
    // until time estimation is up.  And once a log file starts, it should never
    // stop again, and should remain constant.
    // To compare both types of headers, we mutate our saved copy of the header
    // to match the next chunk by updating time if we detect a stopped ->
    // started transition.
    if (monotonic_start_time() == monotonic_clock::min_time) {
      CHECK_EQ(realtime_start_time(), realtime_clock::min_time);
      // We should only be missing the monotonic start time when logging data
      // for remote nodes.  We don't have a good way to determine the remote
      // realtime offset, so it shouldn't be filled out.
      // TODO(austin): If we have a good way, feel free to fill it out.  It
      // probably won't be better than we could do in post though with the same
      // data.
      CHECK(!log_file_header_.mutable_message()->has_realtime_start_time());
      if (new_monotonic_start_time != monotonic_clock::min_time) {
        // If we finally found our start time, update the header.  Do this once
        // because it should never change again.
        log_file_header_.mutable_message()->mutate_monotonic_start_time(
            new_monotonic_start_time.time_since_epoch().count());
        log_file_header_.mutable_message()->mutate_realtime_start_time(
            new_realtime_start_time.time_since_epoch().count());
      }
    }

    // We don't have a good way to set the realtime start time on remote nodes.
    // Confirm it remains consistent.
    CHECK_EQ(log_file_header_.mutable_message()->has_realtime_start_time(),
             message_reader.log_file_header()->has_realtime_start_time());

    // Parts index will *not* match unless we set them to match.  We only want
    // to accept the start time and parts mismatching, so set them.
    log_file_header_.mutable_message()->mutate_parts_index(
        message_reader.log_file_header()->parts_index());

    // Now compare that the headers match.
    if (!CompareFlatBuffer(message_reader.raw_log_file_header(),
                           log_file_header_)) {
      if (message_reader.log_file_header()->has_logger_uuid() &&
          log_file_header_.message().has_logger_uuid() &&
          message_reader.log_file_header()->logger_uuid()->string_view() !=
              log_file_header_.message().logger_uuid()->string_view()) {
        LOG(FATAL) << "Logger UUIDs don't match between log file chunks "
                   << filenames_[0] << " and " << filenames_[i]
                   << ", this is not supported.";
      }
      if (message_reader.log_file_header()->has_parts_uuid() &&
          log_file_header_.message().has_parts_uuid() &&
          message_reader.log_file_header()->parts_uuid()->string_view() !=
              log_file_header_.message().parts_uuid()->string_view()) {
        LOG(FATAL) << "Parts UUIDs don't match between log file chunks "
                   << filenames_[0] << " and " << filenames_[i]
                   << ", this is not supported.";
      }

      LOG(FATAL) << "Header is different between log file chunks "
                 << filenames_[0] << " and " << filenames_[i]
                 << ", this is not supported.";
    }
  }
  // Put the parts index back to the first log file chunk.
  log_file_header_.mutable_message()->mutate_parts_index(
      message_reader_->log_file_header()->parts_index());

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
    // In order for the headers to identically compare, they need to have the
    // same parts_index.  Rewrite the saved header with the new parts_index,
    // compare, and then restore.
    const int32_t original_parts_index =
        log_file_header_.message().parts_index();
    log_file_header_.mutable_message()->mutate_parts_index(
        message_reader_->log_file_header()->parts_index());

    CHECK(CompareFlatBuffer(message_reader_->raw_log_file_header(),
                            log_file_header_))
        << ": Header is different between log file chunks "
        << filenames_[next_filename_index_] << " and "
        << filenames_[next_filename_index_ - 1] << ", this is not supported.";

    log_file_header_.mutable_message()->mutate_parts_index(
        original_parts_index);
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
      VLOG(1) << MaybeNodeName(target_node_) << "All up to date on " << this
              << ", dequeued " << last_dequeued_time << " queue time "
              << time_to_queue_;
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
  VLOG(1) << MaybeNodeName(target_node_) << "Queueing, going until "
          << time_to_queue_ << " " << filename();

  bool was_emplaced = false;
  while (true) {
    // Stop if we have enough.
    if (newest_timestamp() > time_to_queue_ + max_out_of_order_duration() &&
        was_emplaced) {
      VLOG(1) << MaybeNodeName(target_node_) << "Done queueing on " << this
              << ", queued to " << newest_timestamp() << " with requeue time "
              << time_to_queue_;
      return true;
    }

    if (std::optional<FlatbufferVector<MessageHeader>> msg =
            message_reader_->ReadMessage()) {
      const MessageHeader &header = msg.value().message();

      const monotonic_clock::time_point timestamp = monotonic_clock::time_point(
          chrono::nanoseconds(header.monotonic_sent_time()));

      if (VLOG_IS_ON(2)) {
        LOG(INFO) << MaybeNodeName(target_node_) << "Queued " << this << " "
                  << filename() << " ttq: " << time_to_queue_ << " now "
                  << newest_timestamp() << " start time "
                  << monotonic_start_time() << " " << FlatbufferToJson(&header);
      } else if (VLOG_IS_ON(1)) {
        FlatbufferVector<MessageHeader> copy = msg.value();
        copy.mutable_message()->clear_data();
        LOG(INFO) << MaybeNodeName(target_node_) << "Queued " << this << " "
                  << filename() << " ttq: " << time_to_queue_ << " now "
                  << newest_timestamp() << " start time "
                  << monotonic_start_time() << " " << FlatbufferToJson(copy);
      }

      const int channel_index = header.channel_index();
      was_emplaced = channels_to_write_[channel_index]->emplace_back(
          std::move(msg.value()));
      if (was_emplaced) {
        newest_timestamp_ = std::max(newest_timestamp_, timestamp);
      }
    } else {
      if (!NextLogFile()) {
        VLOG(1) << MaybeNodeName(target_node_) << "No more files, last was "
                << filenames_.back();
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
  target_node_ = reinterpreted_target_node;

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
  channels_[channel_index].data.PopFront();

  VLOG(1) << MaybeNodeName(target_node_) << "Popped Data " << this << " "
          << std::get<0>(timestamp) << " for "
          << configuration::StrippedChannelToString(
                 configuration()->channels()->Get(channel_index))
          << " (" << channel_index << ")";

  QueueMessages(std::get<0>(timestamp));

  return std::make_tuple(std::get<0>(timestamp), std::get<1>(timestamp),
                         std::move(front));
}

std::tuple<monotonic_clock::time_point, uint32_t,
           FlatbufferVector<MessageHeader>>
SplitMessageReader::PopOldestTimestamp(int channel, int node_index) {
  CHECK_GT(channels_[channel].timestamps[node_index].size(), 0u);
  const std::tuple<monotonic_clock::time_point, uint32_t, const MessageHeader *>
      timestamp = channels_[channel].timestamps[node_index].front_timestamp();
  FlatbufferVector<MessageHeader> front =
      std::move(channels_[channel].timestamps[node_index].front());
  channels_[channel].timestamps[node_index].PopFront();

  VLOG(1) << MaybeNodeName(target_node_) << "Popped timestamp " << this << " "
          << std::get<0>(timestamp) << " for "
          << configuration::StrippedChannelToString(
                 configuration()->channels()->Get(channel))
          << " on "
          << configuration()->nodes()->Get(node_index)->name()->string_view()
          << " (" << node_index << ")";

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

void SplitMessageReader::MessageHeaderQueue::PopFront() {
  data_.pop_front();
  if (data_.size() != 0u) {
    // Yup, new data.
    if (timestamps) {
      timestamp_merger->UpdateTimestamp(split_reader, front_timestamp());
    } else {
      timestamp_merger->Update(split_reader, front_timestamp());
    }
  } else {
    // Poke anyways to update the heap.
    if (timestamps) {
      timestamp_merger->UpdateTimestamp(
          nullptr, std::make_tuple(monotonic_clock::min_time, 0, nullptr));
    } else {
      timestamp_merger->Update(
          nullptr, std::make_tuple(monotonic_clock::min_time, 0, nullptr));
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
  if (split_message_reader != nullptr) {
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
  }

  // If we are just a data merger, don't wait for timestamps.
  if (!has_timestamps_) {
    if (!message_heap_.empty()) {
      channel_merger_->Update(std::get<0>(message_heap_[0]), channel_index_);
      pushed_ = true;
    } else {
      // Remove ourselves if we are empty.
      channel_merger_->Update(monotonic_clock::min_time, channel_index_);
    }
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
  if (split_message_reader != nullptr) {
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
  }

  // If we are a timestamp merger, don't wait for data.  Missing data will be
  // caught at read time.
  if (has_timestamps_) {
    if (!timestamp_heap_.empty()) {
      channel_merger_->Update(std::get<0>(timestamp_heap_[0]), channel_index_);
      pushed_ = true;
    } else {
      // Remove ourselves if we are empty.
      channel_merger_->Update(monotonic_clock::min_time, channel_index_);
    }
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
  while (!message_heap_.empty()) {
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
                             ->PopOldestTimestamp(channel_index_, node_index_);

  // Confirm that the time we have recorded matches.
  CHECK_EQ(std::get<0>(oldest_timestamp), std::get<0>(oldest_timestamp_reader));
  CHECK_EQ(std::get<1>(oldest_timestamp), std::get<1>(oldest_timestamp_reader));

  // Now, keep reading until we have found all duplicates.
  while (!timestamp_heap_.empty()) {
    // See if it is a duplicate.
    std::tuple<monotonic_clock::time_point, uint32_t, SplitMessageReader *>
        next_oldest_timestamp_reader = timestamp_heap_.front();

    std::tuple<monotonic_clock::time_point, uint32_t, const MessageHeader *>
        next_oldest_timestamp_time =
            std::get<2>(next_oldest_timestamp_reader)
                ->oldest_message(channel_index_, node_index_);

    if (std::get<0>(next_oldest_timestamp_time) ==
            std::get<0>(oldest_timestamp) &&
        std::get<1>(next_oldest_timestamp_time) ==
            std::get<1>(oldest_timestamp)) {
      // Pop the timestamp reader pointer.
      std::pop_heap(timestamp_heap_.begin(), timestamp_heap_.end(),
                    &SplitMessageReaderHeapCompare);
      timestamp_heap_.pop_back();

      // Pop the next oldest timestamp.  This re-pushes any messages from the
      // reader.
      std::tuple<monotonic_clock::time_point, uint32_t,
                 FlatbufferVector<MessageHeader>>
          next_oldest_timestamp =
              std::get<2>(next_oldest_timestamp_reader)
                  ->PopOldestTimestamp(channel_index_, node_index_);

      // And make sure the contents matches in it's entirety.
      CHECK(std::get<2>(oldest_timestamp).span() ==
            std::get<2>(next_oldest_timestamp).span())
          << ": Data at the same timestamp doesn't match, "
          << aos::FlatbufferToJson(std::get<2>(oldest_timestamp)) << " vs "
          << aos::FlatbufferToJson(std::get<2>(next_oldest_timestamp)) << " "
          << absl::BytesToHexString(std::string_view(
                 reinterpret_cast<const char *>(
                     std::get<2>(oldest_timestamp).span().data()),
                 std::get<2>(oldest_timestamp).span().size()))
          << " vs "
          << absl::BytesToHexString(std::string_view(
                 reinterpret_cast<const char *>(
                     std::get<2>(next_oldest_timestamp).span().data()),
                 std::get<2>(next_oldest_timestamp).span().size()));

    } else {
      break;
    }
  }

  return oldest_timestamp;
}

std::tuple<TimestampMerger::DeliveryTimestamp, FlatbufferVector<MessageHeader>>
TimestampMerger::PopOldest() {
  if (has_timestamps_) {
    VLOG(1) << "Looking for matching timestamp for "
            << configuration::StrippedChannelToString(
                   configuration_->channels()->Get(channel_index_))
            << " (" << channel_index_ << ") "
            << " at " << std::get<0>(oldest_timestamp());

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
    if (message_heap_.empty()) {
      LOG(WARNING) << MaybeNodeName(configuration_->nodes()->Get(node_index_))
                   << "No data to match timestamp on "
                   << configuration::CleanedChannelToString(
                          configuration_->channels()->Get(channel_index_))
                   << " (" << channel_index_ << ")";
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
          LOG(WARNING) << configuration_->nodes()
                              ->Get(node_index_)
                              ->name()
                              ->string_view()
                       << " Undelivered message, skipping.  Remote time is "
                       << remote_monotonic_time << " timestamp is "
                       << remote_timestamp_monotonic_time << " on channel "
                       << configuration::StrippedChannelToString(
                              configuration_->channels()->Get(channel_index_))
                       << " (" << channel_index_ << ")";
          PopMessageHeap();
          continue;
        } else if (remote_monotonic_time > remote_timestamp_monotonic_time) {
          LOG(WARNING) << configuration_->nodes()
                              ->Get(node_index_)
                              ->name()
                              ->string_view()
                       << " Data not found.  Remote time should be "
                       << remote_timestamp_monotonic_time
                       << ", message time is " << remote_monotonic_time
                       << " on channel "
                       << configuration::StrippedChannelToString(
                              configuration_->channels()->Get(channel_index_))
                       << " (" << channel_index_ << ")"
                       << (VLOG_IS_ON(1) ? DebugString() : "");
          return std::make_tuple(timestamp,
                                 std::move(std::get<2>(oldest_timestamp)));
        }

        timestamp.monotonic_remote_time = remote_monotonic_time;
      }

      VLOG(1) << "Found matching data "
              << configuration::StrippedChannelToString(
                     configuration_->channels()->Get(channel_index_))
              << " (" << channel_index_ << ")";
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

      return std::make_tuple(timestamp, std::move(std::get<2>(oldest_message)));
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

    return std::make_tuple(timestamp, std::move(std::get<2>(oldest_message)));
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
      log_file_header_(split_message_readers_[0]->raw_log_file_header()) {
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
      // In order to identify which logfile(s) map to the target node, do a
      // logical comparison of the nodes, by confirming that we are either in a
      // single-node setup (where the nodes will both be nullptr) or that the
      // node names match (but the other node fields--e.g., hostname lists--may
      // not).
      const bool both_null =
          reader->node() == nullptr && target_node == nullptr;
      const bool both_have_name =
          (reader->node() != nullptr) && (target_node != nullptr) &&
          (reader->node()->has_name() && target_node->has_name());
      const bool node_names_identical =
          both_have_name && (reader->node()->name()->string_view() ==
                             target_node->name()->string_view());
      if (both_null || node_names_identical) {
        if (!found_node) {
          found_node = true;
          log_file_header_ = CopyFlatBuffer(reader->log_file_header());
          VLOG(1) << "Found log file " << reader->filename() << " with node "
                  << FlatbufferToJson(reader->node()) << " start_time "
                  << monotonic_start_time();
        } else {
          // Find the earliest start time.  That way, if we get a full log file
          // directly from the node, and a partial later, we start with the
          // full.  Update our header to match that.
          const monotonic_clock::time_point new_monotonic_start_time(
              chrono::nanoseconds(
                  reader->log_file_header()->monotonic_start_time()));
          const realtime_clock::time_point new_realtime_start_time(
              chrono::nanoseconds(
                  reader->log_file_header()->realtime_start_time()));

          if (monotonic_start_time() == monotonic_clock::min_time ||
              (new_monotonic_start_time != monotonic_clock::min_time &&
               new_monotonic_start_time < monotonic_start_time())) {
            log_file_header_.mutable_message()->mutate_monotonic_start_time(
                new_monotonic_start_time.time_since_epoch().count());
            log_file_header_.mutable_message()->mutate_realtime_start_time(
                new_realtime_start_time.time_since_epoch().count());
            VLOG(1) << "Updated log file " << reader->filename()
                    << " with node " << FlatbufferToJson(reader->node())
                    << " start_time " << new_monotonic_start_time;
          }
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

monotonic_clock::time_point ChannelMerger::OldestMessageTime() const {
  if (channel_heap_.empty()) {
    return monotonic_clock::max_time;
  }
  return channel_heap_.front().first;
}

void ChannelMerger::PushChannelHeap(monotonic_clock::time_point timestamp,
                                    int channel_index) {
  // Pop and recreate the heap if it has already been pushed.  And since we are
  // pushing again, we don't need to clear pushed.
  if (timestamp_mergers_[channel_index].pushed()) {
    const auto channel_iterator = std::find_if(
        channel_heap_.begin(), channel_heap_.end(),
        [channel_index](const std::pair<monotonic_clock::time_point, int> x) {
          return x.second == channel_index;
        });
    DCHECK(channel_iterator != channel_heap_.end());
    if (std::get<0>(*channel_iterator) == timestamp) {
      // It's already in the heap, in the correct spot, so nothing
      // more for us to do here.
      return;
    }
    channel_heap_.erase(channel_iterator);
    std::make_heap(channel_heap_.begin(), channel_heap_.end(),
                   ChannelHeapCompare);
  }

  if (timestamp == monotonic_clock::min_time) {
    timestamp_mergers_[channel_index].set_pushed(false);
    return;
  }

  channel_heap_.push_back(std::make_pair(timestamp, channel_index));

  // The default sort puts the newest message first.  Use a custom comparator to
  // put the oldest message first.
  std::push_heap(channel_heap_.begin(), channel_heap_.end(),
                 ChannelHeapCompare);
}

void ChannelMerger::VerifyHeaps() {
  std::vector<std::pair<monotonic_clock::time_point, int>> channel_heap =
      channel_heap_;
  std::make_heap(channel_heap.begin(), channel_heap.end(), &ChannelHeapCompare);

  for (size_t i = 0; i < channel_heap_.size(); ++i) {
    CHECK(channel_heap_[i] == channel_heap[i]) << ": Heaps diverged...";
    CHECK_EQ(
        std::get<0>(channel_heap[i]),
        timestamp_mergers_[std::get<1>(channel_heap[i])].channel_merger_time());
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

  // Merger handles any queueing needed from here.
  std::tuple<TimestampMerger::DeliveryTimestamp,
             FlatbufferVector<MessageHeader>>
      message = merger->PopOldest();
  DCHECK_EQ(std::get<0>(message).monotonic_event_time,
            oldest_channel_data.first)
      << ": channel_heap_ was corrupted for " << channel_index << ": "
      << DebugString();

  CHECK_GE(std::get<0>(message).monotonic_event_time, last_popped_time_)
      << ": " << MaybeNodeName(log_file_header()->node())
      << "Messages came off the queue out of order. " << DebugString();
  last_popped_time_ = std::get<0>(message).monotonic_event_time;

  VLOG(1) << "Popped " << last_popped_time_ << " "
          << configuration::StrippedChannelToString(
                 configuration()->channels()->Get(channel_index))
          << " (" << channel_index << ")";

  return std::make_tuple(std::get<0>(message), channel_index,
                         std::move(std::get<1>(message)));
}

std::string SplitMessageReader::MessageHeaderQueue::DebugString() const {
  std::stringstream ss;
  for (size_t i = 0; i < data_.size(); ++i) {
    if (i < 5 || i + 5 > data_.size()) {
      if (timestamps) {
        ss << "        msg: ";
      } else {
        ss << "        timestamp: ";
      }
      ss << monotonic_clock::time_point(
                chrono::nanoseconds(data_[i].message().monotonic_sent_time()))
         << " ("
         << realtime_clock::time_point(
                chrono::nanoseconds(data_[i].message().realtime_sent_time()))
         << ") " << data_[i].message().queue_index();
      if (timestamps) {
        ss << "  <- remote "
           << monotonic_clock::time_point(chrono::nanoseconds(
                  data_[i].message().monotonic_remote_time()))
           << " ("
           << realtime_clock::time_point(chrono::nanoseconds(
                  data_[i].message().realtime_remote_time()))
           << ")";
      }
      ss << "\n";
    } else if (i == 5) {
      ss << "        ...\n";
    }
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
    while (!message_heap.empty()) {
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
  while (!channel_heap.empty()) {
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

std::string MaybeNodeName(const Node *node) {
  if (node != nullptr) {
    return node->name()->str() + " ";
  }
  return "";
}

}  // namespace aos::logger
