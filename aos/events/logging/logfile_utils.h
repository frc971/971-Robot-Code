#ifndef AOS_EVENTS_LOGGING_LOGFILE_UTILS_H_
#define AOS_EVENTS_LOGGING_LOGFILE_UTILS_H_

#include <sys/uio.h>

#include <chrono>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/container/btree_set.h"
#include "absl/types/span.h"
#include "aos/containers/resizeable_buffer.h"
#include "aos/events/event_loop.h"
#include "aos/events/logging/boot_timestamp.h"
#include "aos/events/logging/buffer_encoder.h"
#include "aos/events/logging/log_backend.h"
#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/flatbuffers.h"
#include "aos/network/remote_message_generated.h"
#include "flatbuffers/flatbuffers.h"

namespace aos::logger {

enum class LogType : uint8_t {
  // The message originated on this node and should be logged here.
  kLogMessage,
  // The message originated on another node, but only the delivery times are
  // logged here.
  kLogDeliveryTimeOnly,
  // The message originated on another node. Log it and the delivery times
  // together.  The message_gateway is responsible for logging any messages
  // which didn't get delivered.
  kLogMessageAndDeliveryTime,
  // The message originated on the other node and should be logged on this node.
  kLogRemoteMessage
};

// This class manages efficiently writing a sequence of detached buffers to a
// file.  It encodes them, queues them up, and batches the write operation.

class DetachedBufferWriter {
 public:
  // Marker struct for one of our constructor overloads.
  struct already_out_of_space_t {};

  DetachedBufferWriter(std::unique_ptr<FileHandler> file_handler,
                       std::unique_ptr<DataEncoder> encoder);
  // Creates a dummy instance which won't even open a file. It will act as if
  // opening the file ran out of space immediately.
  DetachedBufferWriter(already_out_of_space_t) : ran_out_of_space_(true) {}
  DetachedBufferWriter(DetachedBufferWriter &&other);
  DetachedBufferWriter(const DetachedBufferWriter &) = delete;

  ~DetachedBufferWriter();

  DetachedBufferWriter &operator=(DetachedBufferWriter &&other);
  DetachedBufferWriter &operator=(const DetachedBufferWriter &) = delete;

  std::string_view filename() const { return file_handler_->filename(); }

  // This will be true until Close() is called, unless the file couldn't be
  // created due to running out of space.
  bool is_open() const { return file_handler_->is_open(); }

  // Queues up a finished FlatBufferBuilder to be encoded and written.
  //
  // Triggers a flush if there's enough data queued up.
  //
  // Steals the detached buffer from it.
  void CopyMessage(DataEncoder::Copier *coppier,
                   aos::monotonic_clock::time_point now);

  // Indicates we got ENOSPC when trying to write. After this returns true, no
  // further data is written.
  bool ran_out_of_space() const { return ran_out_of_space_; }

  // To avoid silently failing to write logfiles, you must call this before
  // destruction if ran_out_of_space() is true and the situation has been
  // handled.
  void acknowledge_out_of_space() {
    CHECK(ran_out_of_space_);
    acknowledge_ran_out_of_space_ = true;
  }

  // Fully flushes and closes the underlying file now. No additional data may be
  // enqueued after calling this.
  //
  // This will be performed in the destructor automatically.
  //
  // Note that this may set ran_out_of_space().
  void Close();

  // Returns the total number of bytes written and currently queued.
  size_t total_bytes() const {
    if (!encoder_) {
      return 0;
    }
    return encoder_->total_bytes();
  }

  WriteStats* WriteStatistics() const { return file_handler_->WriteStatistics(); }

 private:
  // Performs a single writev call with as much of the data we have queued up as
  // possible.  now is the time we flushed at, to be recorded in
  // last_flush_time_.
  //
  // This will normally take all of the data we have queued up, unless an
  // encoder has spit out a big enough chunk all at once that we can't manage
  // all of it.
  void Flush(aos::monotonic_clock::time_point now);

  // Flushes data if we've reached the threshold to do that as part of normal
  // operation either due to the outstanding queued data, or because we have
  // passed our flush period.  now is the current time to save some CPU grabbing
  // the current time.  It just needs to be close.
  void FlushAtThreshold(aos::monotonic_clock::time_point now);

  std::unique_ptr<FileHandler> file_handler_;
  std::unique_ptr<DataEncoder> encoder_;

  bool ran_out_of_space_ = false;
  bool acknowledge_ran_out_of_space_ = false;

  aos::monotonic_clock::time_point last_flush_time_ =
      aos::monotonic_clock::min_time;
};

// Specialized writer to single file
class DetachedBufferFileWriter : public FileBackend,
                                 public DetachedBufferWriter {
 public:
  DetachedBufferFileWriter(std::string_view filename,
                           std::unique_ptr<DataEncoder> encoder)
      : FileBackend("/"),
        DetachedBufferWriter(FileBackend::RequestFile(filename),
                             std::move(encoder)) {}
};

// Repacks the provided RemoteMessage into fbb.
flatbuffers::Offset<MessageHeader> PackRemoteMessage(
    flatbuffers::FlatBufferBuilder *fbb,
    const message_bridge::RemoteMessage *msg, int channel_index,
    const aos::monotonic_clock::time_point monotonic_timestamp_time);

constexpr flatbuffers::uoffset_t PackRemoteMessageSize() { return 96u; }
size_t PackRemoteMessageInline(
    uint8_t *data, const message_bridge::RemoteMessage *msg, int channel_index,
    const aos::monotonic_clock::time_point monotonic_timestamp_time,
    size_t start_byte, size_t end_byte);

// Packes a message pointed to by the context into a MessageHeader.
flatbuffers::Offset<MessageHeader> PackMessage(
    flatbuffers::FlatBufferBuilder *fbb, const Context &context,
    int channel_index, LogType log_type);

// Returns the size that the packed message from PackMessage or
// PackMessageInline will be.
flatbuffers::uoffset_t PackMessageSize(LogType log_type, size_t data_size);

// Packs the provided message pointed to by context into the provided buffer.
// This is equivalent to PackMessage, but doesn't require allocating a
// FlatBufferBuilder underneath.
size_t PackMessageInline(uint8_t *data, const Context &contex,
                         int channel_index, LogType log_type, size_t start_byte,
                         size_t end_byte);

// Class to read chunks out of a log file.
class SpanReader {
 public:
  SpanReader(std::string_view filename, bool quiet = false);

  std::string_view filename() const { return filename_; }

  size_t TotalRead() const { return total_read_; }
  size_t TotalConsumed() const { return total_consumed_; }
  bool IsIncomplete() const {
    return is_finished_ && total_consumed_ < total_read_;
  }

  // Returns a span with the data for the next message from the log file,
  // including the size.  The result is only guarenteed to be valid until
  // ReadMessage() or PeekMessage() is called again.
  absl::Span<const uint8_t> ReadMessage();

  // Returns a span with the data for the next message without consuming it.
  // Multiple calls to PeekMessage return the same data.  ReadMessage or
  // ConsumeMessage must be called to get the next message.
  absl::Span<const uint8_t> PeekMessage();
  // Consumes the message so the next call to ReadMessage or PeekMessage returns
  // new data.  This does not invalidate the data.
  void ConsumeMessage();

 private:
  // TODO(austin): Optimization:
  //   Allocate the 256k blocks like we do today.  But, refcount them with
  //   shared_ptr pointed to by the messageheader that is returned.  This avoids
  //   the copy.  Need to do more benchmarking.
  //   And (Brian): Consider just mmapping the file and handing out refcounted
  //   pointers into that too.

  // Reads a chunk of data into data_.  Returns false if no data was read.
  bool ReadBlock();

  std::string filename_;

  // File reader and data decoder.
  std::unique_ptr<DataDecoder> decoder_;

  // Vector to read into.
  ResizeableBuffer data_;

  // Amount of data consumed already in data_.
  size_t consumed_data_ = 0;

  // Accumulates the total volume of bytes read from filename_
  size_t total_read_ = 0;

  // Accumulates the total volume of read bytes that were 'consumed' into
  // messages. May be less than total_read_, if the last message (span) is
  // either truncated or somehow corrupt.
  size_t total_consumed_ = 0;

  // Reached the end, no more readable messages.
  bool is_finished_ = false;
};

// Reads the last header from a log file.  This handles any duplicate headers
// that were written.
std::optional<SizePrefixedFlatbufferVector<LogFileHeader>> ReadHeader(
    SpanReader *span_reader);
std::optional<SizePrefixedFlatbufferVector<LogFileHeader>> ReadHeader(
    std::string_view filename);
// Reads the Nth message from a log file, excluding the header.  Note: this
// doesn't handle duplicate headers.
std::optional<SizePrefixedFlatbufferVector<MessageHeader>> ReadNthMessage(
    std::string_view filename, size_t n);

class UnpackedMessageHeader;

// Class which handles reading the header and messages from the log file.  This
// handles any per-file state left before merging below.
class MessageReader {
 public:
  MessageReader(std::string_view filename);

  std::string_view filename() const { return span_reader_.filename(); }

  // Returns the header from the log file.
  const LogFileHeader *log_file_header() const {
    return &raw_log_file_header_.message();
  }

  // Returns the raw data of the header from the log file.
  const SizePrefixedFlatbufferVector<LogFileHeader> &raw_log_file_header()
      const {
    return raw_log_file_header_;
  }

  // Returns the minimum maount of data needed to queue up for sorting before
  // ware guarenteed to not see data out of order.
  std::chrono::nanoseconds max_out_of_order_duration() const {
    return max_out_of_order_duration_;
  }

  // Returns the newest timestamp read out of the log file.
  monotonic_clock::time_point newest_timestamp() const {
    return newest_timestamp_;
  }

  // Returns the next message if there is one.
  std::shared_ptr<UnpackedMessageHeader> ReadMessage();

  // The time at which we need to read another chunk from the logfile.
  monotonic_clock::time_point queue_data_time() const {
    return newest_timestamp() - max_out_of_order_duration();
  }

  // Flag value setters for testing
  void set_crash_on_corrupt_message_flag(bool b) {
    crash_on_corrupt_message_flag_ = b;
  }
  void set_ignore_corrupt_messages_flag(bool b) {
    ignore_corrupt_messages_flag_ = b;
  }

 private:
  // Log chunk reader.
  SpanReader span_reader_;

  // Vector holding the raw data for the log file header.
  SizePrefixedFlatbufferVector<LogFileHeader> raw_log_file_header_;

  // Minimum amount of data to queue up for sorting before we are guarenteed
  // to not see data out of order.
  std::chrono::nanoseconds max_out_of_order_duration_;

  // Timestamp of the newest message in a channel queue.
  monotonic_clock::time_point newest_timestamp_ = monotonic_clock::min_time;

  // Total volume of verifiable messages from the beginning of the file.
  // TODO - are message counts also useful?
  size_t total_verified_before_ = 0;

  // Total volume of messages with corrupted flatbuffer formatting, if any.
  // Excludes corrupted message content.
  // TODO - if the layout included something as simple as a CRC (relatively
  // fast and robust enough) for each span, then corrupted content could be
  // included in this check.
  size_t total_corrupted_ = 0;

  // Total volume of verifiable messages intermixed with corrupted messages,
  // if any. Will be == 0 if total_corrupted_ == 0.
  size_t total_verified_during_ = 0;

  // Total volume of verifiable messages found after the last corrupted one,
  // if any. Will be == 0 if total_corrupted_ == 0.
  size_t total_verified_after_ = 0;

  bool is_corrupted() const { return total_corrupted_ > 0; }

  bool crash_on_corrupt_message_flag_ = true;
  bool ignore_corrupt_messages_flag_ = false;
};

// A class to seamlessly read messages from a list of part files.
class PartsMessageReader {
 public:
  PartsMessageReader(LogParts log_parts);

  std::string_view filename() const { return message_reader_.filename(); }

  // Returns the LogParts that holds the filenames we are reading.
  const LogParts &parts() const { return parts_; }

  const LogFileHeader *log_file_header() const {
    return message_reader_.log_file_header();
  }

  // Returns the minimum amount of data needed to queue up for sorting before
  // we are guarenteed to not see data out of order.
  std::chrono::nanoseconds max_out_of_order_duration() const {
    return message_reader_.max_out_of_order_duration();
  }

  // Returns the newest timestamp read out of the log file.
  monotonic_clock::time_point newest_timestamp() const {
    return newest_timestamp_;
  }

  // Returns the next message if there is one, or nullopt if we have reached the
  // end of all the files.
  // Note: reading the next message may change the max_out_of_order_duration().
  std::shared_ptr<UnpackedMessageHeader> ReadMessage();

  // Returns the boot count for the requested node, or std::nullopt if we don't
  // know.
  std::optional<size_t> boot_count(size_t node_index) const {
    CHECK_GE(node_index, 0u);
    CHECK_LT(node_index, boot_counts_.size());
    return boot_counts_[node_index];
  }

 private:
  // Opens the next log and updates message_reader_.  Sets done_ if there is
  // nothing more to do.
  void NextLog();
  void ComputeBootCounts();

  const LogParts parts_;
  size_t next_part_index_ = 1u;
  bool done_ = false;
  MessageReader message_reader_;
  // We instantiate the next one early, to allow implementations to prefetch.
  // TODO(Brian): To get optimal performance when downloading, this needs more
  // communication with the implementation to prioritize the next part and add
  // more parallelism when it helps. Maybe some kind of a queue of parts in
  // order, and the implementation gets to pull however many make sense off the
  // front?
  std::optional<MessageReader> next_message_reader_;

  // True after we have seen a message after the start of the log.  The
  // guarentees on logging essentially are that all data from before the
  // starting time of the log may be arbitrarily out of order, but once we get
  // max_out_of_order_duration past the start, everything will remain within
  // max_out_of_order_duration.  We shouldn't see anything before the start
  // after we've seen a message that is at least max_out_of_order_duration after
  // the start.
  bool after_start_ = false;

  monotonic_clock::time_point newest_timestamp_ = monotonic_clock::min_time;

  // Per node boot counts.
  std::vector<std::optional<size_t>> boot_counts_;
};

// Stores MessageHeader as a flat header and inline, aligned block of data.
class UnpackedMessageHeader {
 public:
  UnpackedMessageHeader(
      uint32_t channel_index, monotonic_clock::time_point monotonic_sent_time,
      realtime_clock::time_point realtime_sent_time, uint32_t queue_index,
      std::optional<monotonic_clock::time_point> monotonic_remote_time,
      std::optional<realtime_clock::time_point> realtime_remote_time,
      std::optional<uint32_t> remote_queue_index,
      monotonic_clock::time_point monotonic_timestamp_time,
      bool has_monotonic_timestamp_time, absl::Span<const uint8_t> span)
      : channel_index(channel_index),
        monotonic_sent_time(monotonic_sent_time),
        realtime_sent_time(realtime_sent_time),
        queue_index(queue_index),
        monotonic_remote_time(monotonic_remote_time),
        realtime_remote_time(realtime_remote_time),
        remote_queue_index(remote_queue_index),
        monotonic_timestamp_time(monotonic_timestamp_time),
        has_monotonic_timestamp_time(has_monotonic_timestamp_time),
        span(span) {}
  UnpackedMessageHeader(const UnpackedMessageHeader &) = delete;
  UnpackedMessageHeader &operator=(const UnpackedMessageHeader &) = delete;

  // The channel.
  uint32_t channel_index = 0xffffffff;

  monotonic_clock::time_point monotonic_sent_time;
  realtime_clock::time_point realtime_sent_time;

  // The local queue index.
  uint32_t queue_index = 0xffffffff;

  std::optional<aos::monotonic_clock::time_point> monotonic_remote_time;

  std::optional<realtime_clock::time_point> realtime_remote_time;
  std::optional<uint32_t> remote_queue_index;

  // This field is defaulted in the flatbuffer, so we need to store both the
  // possibly defaulted value and whether it is defaulted.
  monotonic_clock::time_point monotonic_timestamp_time;
  bool has_monotonic_timestamp_time;

  static std::shared_ptr<UnpackedMessageHeader> MakeMessage(
      const MessageHeader &message);

  // Note: we are storing a span here because we need something to put in the
  // SharedSpan pointer that RawSender takes.  We are using the aliasing
  // constructor of shared_ptr to avoid the allocation, and it needs a nice
  // pointer to track.
  absl::Span<const uint8_t> span;

  char actual_data[];

 private:
  ~UnpackedMessageHeader() {}

  static void DestroyAndFree(UnpackedMessageHeader *p) {
    p->~UnpackedMessageHeader();
    free(p);
  }
};

std::ostream &operator<<(std::ostream &os,
                         const UnpackedMessageHeader &message);

// Struct to hold a message as it gets sorted on a single node.
struct Message {
  // The channel.
  uint32_t channel_index = 0xffffffff;
  // The local queue index.
  // TODO(austin): Technically the boot inside queue_index is redundant with
  // timestamp.  In practice, it is less error-prone to duplicate it.  Maybe a
  // function to return the combined struct?
  BootQueueIndex queue_index;
  // The local timestamp.
  BootTimestamp timestamp;

  // Remote boot when this is a timestamp.
  size_t monotonic_remote_boot = 0xffffff;

  size_t monotonic_timestamp_boot = 0xffffff;

  std::shared_ptr<UnpackedMessageHeader> data;

  bool operator<(const Message &m2) const;
  bool operator>=(const Message &m2) const;
  bool operator==(const Message &m2) const;
};

std::ostream &operator<<(std::ostream &os, const Message &m);

// Structure to hold a full message and all the timestamps, which may or may not
// have been sent from a remote node.  The remote_queue_index will be invalid if
// this message is from the point of view of the node which sent it.
struct TimestampedMessage {
  uint32_t channel_index = 0xffffffff;

  BootQueueIndex queue_index;
  BootTimestamp monotonic_event_time;
  realtime_clock::time_point realtime_event_time = realtime_clock::min_time;

  BootQueueIndex remote_queue_index;
  BootTimestamp monotonic_remote_time;
  realtime_clock::time_point realtime_remote_time = realtime_clock::min_time;

  BootTimestamp monotonic_timestamp_time;

  std::shared_ptr<UnpackedMessageHeader> data;
};

std::ostream &operator<<(std::ostream &os, const TimestampedMessage &m);

// Class to sort the resulting messages from a PartsMessageReader.
class LogPartsSorter {
 public:
  LogPartsSorter(LogParts log_parts);

  // Returns the parts that this is sorting messages from.
  const LogParts &parts() const { return parts_message_reader_.parts(); }

  monotonic_clock::time_point monotonic_start_time() const {
    return parts().monotonic_start_time;
  }
  realtime_clock::time_point realtime_start_time() const {
    return parts().realtime_start_time;
  }

  // The time this data is sorted until.
  monotonic_clock::time_point sorted_until() const { return sorted_until_; }

  // Returns the next sorted message from the log file.  It is safe to call
  // std::move() on the result to move the data flatbuffer from it.
  Message *Front();
  // Pops the front message.  This should only be called after a call to
  // Front().
  void PopFront();

  // Returns a debug string representing the contents of this sorter.
  std::string DebugString() const;

 private:
  // Log parts reader we are wrapping.
  PartsMessageReader parts_message_reader_;
  // Cache of the time we are sorted until.
  aos::monotonic_clock::time_point sorted_until_ = monotonic_clock::min_time;

  // Timestamp of the last message returned.  Used to make sure nothing goes
  // backwards.
  monotonic_clock::time_point last_message_time_ = monotonic_clock::min_time;

  // Set used for efficient sorting of messages.  We can benchmark and evaluate
  // other data structures if this proves to be the bottleneck.
  absl::btree_set<Message> messages_;

  // Mapping from channel to source node.
  // TODO(austin): Should we put this in Boots so it can be cached for everyone?
  std::vector<size_t> source_node_index_;
};

// Class to run merge sort on the messages from multiple LogPartsSorter
// instances.
class NodeMerger {
 public:
  NodeMerger(std::vector<LogParts> parts);

  // Copying and moving will mess up the internal raw pointers.  Just don't do
  // it.
  NodeMerger(NodeMerger const &) = delete;
  NodeMerger(NodeMerger &&) = delete;
  void operator=(NodeMerger const &) = delete;
  void operator=(NodeMerger &&) = delete;

  // Node index in the configuration of this node.
  int node() const { return node_; }

  // List of parts being sorted together.
  std::vector<const LogParts *> Parts() const;

  const Configuration *configuration() const {
    return parts_sorters_[0].parts().config.get();
  }

  monotonic_clock::time_point monotonic_start_time() const {
    return monotonic_start_time_;
  }
  realtime_clock::time_point realtime_start_time() const {
    return realtime_start_time_;
  }
  monotonic_clock::time_point monotonic_oldest_time() const {
    return monotonic_oldest_time_;
  }

  // The time this data is sorted until.
  monotonic_clock::time_point sorted_until() const { return sorted_until_; }

  // Returns the next sorted message from the set of log files.  It is safe to
  // call std::move() on the result to move the data flatbuffer from it.
  Message *Front();
  // Pops the front message.  This should only be called after a call to
  // Front().
  void PopFront();

 private:
  // Unsorted list of all parts sorters.
  std::vector<LogPartsSorter> parts_sorters_;
  // Pointer to the parts sorter holding the current Front message if one
  // exists, or nullptr if a new one needs to be found.
  LogPartsSorter *current_ = nullptr;
  // Cached sorted_until value.
  aos::monotonic_clock::time_point sorted_until_ = monotonic_clock::min_time;

  // Cached node.
  int node_;

  // Timestamp of the last message returned.  Used to make sure nothing goes
  // backwards.
  monotonic_clock::time_point last_message_time_ = monotonic_clock::min_time;

  realtime_clock::time_point realtime_start_time_ = realtime_clock::max_time;
  monotonic_clock::time_point monotonic_start_time_ = monotonic_clock::max_time;
  monotonic_clock::time_point monotonic_oldest_time_ =
      monotonic_clock::max_time;
};

// Class to concatenate multiple boots worth of logs into a single per-node
// stream.
class BootMerger {
 public:
  BootMerger(std::vector<LogParts> file);

  // Copying and moving will mess up the internal raw pointers.  Just don't do
  // it.
  BootMerger(BootMerger const &) = delete;
  BootMerger(BootMerger &&) = delete;
  void operator=(BootMerger const &) = delete;
  void operator=(BootMerger &&) = delete;

  // Node index in the configuration of this node.
  int node() const { return node_mergers_[0]->node(); }

  // List of parts being sorted together.
  std::vector<const LogParts *> Parts() const;

  const Configuration *configuration() const {
    return node_mergers_[0]->configuration();
  }

  monotonic_clock::time_point monotonic_start_time(size_t boot) const {
    CHECK_LT(boot, node_mergers_.size());
    return node_mergers_[boot]->monotonic_start_time();
  }
  realtime_clock::time_point realtime_start_time(size_t boot) const {
    CHECK_LT(boot, node_mergers_.size());
    return node_mergers_[boot]->realtime_start_time();
  }
  monotonic_clock::time_point monotonic_oldest_time(size_t boot) const {
    CHECK_LT(boot, node_mergers_.size());
    return node_mergers_[boot]->monotonic_oldest_time();
  }

  bool started() const {
    return node_mergers_[index_]->sorted_until() != monotonic_clock::min_time ||
           index_ != 0;
  }

  // Returns the next sorted message from the set of log files.  It is safe to
  // call std::move() on the result to move the data flatbuffer from it.
  Message *Front();
  // Pops the front message.  This should only be called after a call to
  // Front().
  void PopFront();

 private:
  int index_ = 0;

  // TODO(austin): Sanjay points out this is pretty inefficient.  Don't keep so
  // many things open.
  std::vector<std::unique_ptr<NodeMerger>> node_mergers_;
};

// Class to match timestamps with the corresponding data from other nodes.
//
// This class also buffers data for the node it represents, and supports
// notifying when new data is queued as well as queueing until a point in time.
class TimestampMapper {
 public:
  TimestampMapper(std::vector<LogParts> file);

  // Copying and moving will mess up the internal raw pointers.  Just don't do
  // it.
  TimestampMapper(TimestampMapper const &) = delete;
  TimestampMapper(TimestampMapper &&) = delete;
  void operator=(TimestampMapper const &) = delete;
  void operator=(TimestampMapper &&) = delete;

  // TODO(austin): It would be super helpful to provide a way to queue up to
  // time X without matching timestamps, and to then be able to pull the
  // timestamps out of this queue.  This lets us bootstrap time estimation
  // without exploding memory usage worst case.

  const Configuration *configuration() const { return configuration_.get(); }

  // Returns which node this is sorting for.
  size_t node() const { return boot_merger_.node(); }

  // The start time of this log.
  monotonic_clock::time_point monotonic_start_time(size_t boot) const {
    return boot_merger_.monotonic_start_time(boot);
  }
  realtime_clock::time_point realtime_start_time(size_t boot) const {
    return boot_merger_.realtime_start_time(boot);
  }
  // Returns the oldest timestamp on a message on this boot.
  monotonic_clock::time_point monotonic_oldest_time(size_t boot) const {
    return boot_merger_.monotonic_oldest_time(boot);
  }

  // Uses timestamp_mapper as the peer for its node. Only one mapper may be set
  // for each node.  Peers are used to look up the data for timestamps on this
  // node.
  void AddPeer(TimestampMapper *timestamp_mapper);

  // Returns true if anything has been queued up.
  bool started() const { return boot_merger_.started(); }

  // Returns the next message for this node.
  TimestampedMessage *Front();
  // Pops the next message.  Front must be called first.
  void PopFront();

  // Returns debug information about this node.
  std::string DebugString() const;

  // Queues data the provided time.
  void QueueUntil(BootTimestamp queue_time);
  // Queues until we have time_estimation_buffer of data in the queue.
  void QueueFor(std::chrono::nanoseconds time_estimation_buffer);

  // Queues until the condition is met.
  template <typename T>
  void QueueUntilCondition(T fn) {
    while (true) {
      if (fn()) {
        break;
      }
      if (!QueueMatched()) {
        break;
      }
    }
  }

  // Sets the callback that can be used to skip messages.
  void set_replay_channels_callback(
      std::function<bool(const TimestampedMessage &)> fn) {
    replay_channels_callback_ = fn;
  }

  // Sets a callback to be called whenever a full message is queued.
  void set_timestamp_callback(std::function<void(TimestampedMessage *)> fn) {
    timestamp_callback_ = fn;
  }

 private:
  // Result of MaybeQueueMatched
  enum class MatchResult : uint8_t {
    kEndOfFile,  // End of the log file being read
    kQueued,     // Message was queued
    kSkipped     // Message was skipped over
  };

  // The state for a remote node.  This holds the data that needs to be matched
  // with the remote node's timestamps.
  struct NodeData {
    // True if we should save data here.  This should be true if any of the
    // bools in delivered below are true.
    bool any_delivered = false;

    // True if we have a peer and therefore should be saving data for it.
    bool save_for_peer = false;

    // Peer pointer.  This node is only to be considered if a peer is set.
    TimestampMapper *peer = nullptr;

    struct ChannelData {
      // Deque per channel.  This contains the data from the outside
      // TimestampMapper node which is relevant for the node this NodeData
      // points to.
      std::deque<Message> messages;
      // Bool tracking per channel if a message is delivered to the node this
      // NodeData represents.
      bool delivered = false;
      // The TTL for delivery.
      std::chrono::nanoseconds time_to_live = std::chrono::nanoseconds(0);
    };

    // Vector with per channel data.
    std::vector<ChannelData> channels;
  };

  // Returns (and forgets about) the data for the provided timestamp message
  // showing when it was delivered to this node.
  Message MatchingMessageFor(const Message &message);

  // Queues up a single message into our message queue, and any nodes that this
  // message is delivered to.  Returns true if one was available, false
  // otherwise.
  bool Queue();

  // Queues up a single matched message into our matched message queue.  Returns
  // true if one was queued, and false otherwise.
  bool QueueMatched();

  // Queues a message if the replay_channels_callback is passed and the end of
  // the log file has not been reached.
  MatchResult MaybeQueueMatched();

  // Queues up data until we have at least one message >= to time t.
  // Useful for triggering a remote node to read enough data to have the
  // timestamp you care about available.
  void QueueUnmatchedUntil(BootTimestamp t);

  // Queues m into matched_messages_.
  void QueueMessage(Message *m);

  // If a replay_channels_callback was set and the callback returns false, a
  // matched message is popped and true is returned. Otherwise false is
  // returned.
  bool CheckReplayChannelsAndMaybePop(const TimestampedMessage &message);

  // Returns the name of the node this class is sorting for.
  std::string_view node_name() const {
    return configuration_->has_nodes() ? configuration_->nodes()
                                             ->Get(boot_merger_.node())
                                             ->name()
                                             ->string_view()
                                       : "(single node)";
  }

  // The node merger to source messages from.
  BootMerger boot_merger_;

  std::shared_ptr<const Configuration> configuration_;

  // The buffer of messages for this node.  These are not matched with any
  // remote data.
  std::deque<Message> messages_;
  // The node index for the source node for each channel.
  std::vector<size_t> source_node_;

  // Vector per node.  Not all nodes will have anything.
  std::vector<NodeData> nodes_data_;

  // Latest message to return.
  std::deque<TimestampedMessage> matched_messages_;

  // Tracks the state of the first message in matched_messages_.  Do we need to
  // update it, is it valid, or should we return nullptr?
  enum class FirstMessage {
    kNeedsUpdate,
    kInMessage,
    kNullptr,
  };
  FirstMessage first_message_ = FirstMessage::kNeedsUpdate;

  // Timestamp of the last message returned.  Used to make sure nothing goes
  // backwards.
  BootTimestamp last_message_time_ = BootTimestamp::min_time();
  BootTimestamp last_popped_message_time_ = BootTimestamp::min_time();
  // Time this node is queued up until.  Used for caching.
  BootTimestamp queued_until_ = BootTimestamp::min_time();

  std::function<void(TimestampedMessage *)> timestamp_callback_;
  std::function<bool(TimestampedMessage &)> replay_channels_callback_;
};

// Returns the node name with a trailing space, or an empty string if we are on
// a single node.
std::string MaybeNodeName(const Node *);

// Class to copy a RemoteMessage into the provided buffer.
class RemoteMessageCopier : public DataEncoder::Copier {
 public:
  RemoteMessageCopier(const message_bridge::RemoteMessage *message,
                      int channel_index,
                      aos::monotonic_clock::time_point monotonic_timestamp_time,
                      EventLoop *event_loop)
      : DataEncoder::Copier(PackRemoteMessageSize()),
        message_(message),
        channel_index_(channel_index),
        monotonic_timestamp_time_(monotonic_timestamp_time),
        event_loop_(event_loop) {}

  monotonic_clock::time_point end_time() const { return end_time_; }

  size_t Copy(uint8_t *data, size_t start_byte, size_t end_byte) final {
    size_t result = PackRemoteMessageInline(data, message_, channel_index_,
                                            monotonic_timestamp_time_,
                                            start_byte, end_byte);
    end_time_ = event_loop_->monotonic_now();
    return result;
  }

 private:
  const message_bridge::RemoteMessage *message_;
  int channel_index_;
  aos::monotonic_clock::time_point monotonic_timestamp_time_;
  EventLoop *event_loop_;
  monotonic_clock::time_point end_time_;
};

// Class to copy a context into the provided buffer.
class ContextDataCopier : public DataEncoder::Copier {
 public:
  ContextDataCopier(const Context &context, int channel_index, LogType log_type,
                    EventLoop *event_loop)
      : DataEncoder::Copier(PackMessageSize(log_type, context.size)),
        context_(context),
        channel_index_(channel_index),
        log_type_(log_type),
        event_loop_(event_loop) {}

  monotonic_clock::time_point end_time() const { return end_time_; }

  size_t Copy(uint8_t *data, size_t start_byte, size_t end_byte) final {
    size_t result = PackMessageInline(data, context_, channel_index_, log_type_,
                                      start_byte, end_byte);
    end_time_ = event_loop_->monotonic_now();
    return result;
  }

 private:
  const Context &context_;
  const int channel_index_;
  const LogType log_type_;
  EventLoop *event_loop_;
  monotonic_clock::time_point end_time_;
};

}  // namespace aos::logger

#endif  // AOS_EVENTS_LOGGING_LOGFILE_UTILS_H_
