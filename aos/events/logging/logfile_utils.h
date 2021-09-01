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
#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/flatbuffers.h"
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

  DetachedBufferWriter(std::string_view filename,
                       std::unique_ptr<DetachedBufferEncoder> encoder);
  // Creates a dummy instance which won't even open a file. It will act as if
  // opening the file ran out of space immediately.
  DetachedBufferWriter(already_out_of_space_t) : ran_out_of_space_(true) {}
  DetachedBufferWriter(DetachedBufferWriter &&other);
  DetachedBufferWriter(const DetachedBufferWriter &) = delete;

  ~DetachedBufferWriter();

  DetachedBufferWriter &operator=(DetachedBufferWriter &&other);
  DetachedBufferWriter &operator=(const DetachedBufferWriter &) = delete;

  std::string_view filename() const { return filename_; }

  // This will be true until Close() is called, unless the file couldn't be
  // created due to running out of space.
  bool is_open() const { return fd_ != -1; }

  // Queues up a finished FlatBufferBuilder to be encoded and written.
  //
  // Triggers a flush if there's enough data queued up.
  //
  // Steals the detached buffer from it.
  void QueueSizedFlatbuffer(flatbuffers::FlatBufferBuilder *fbb,
                            aos::monotonic_clock::time_point now) {
    QueueSizedFlatbuffer(fbb->Release(), now);
  }
  // May steal the backing storage of buffer, or may leave it alone.
  void QueueSizedFlatbuffer(flatbuffers::DetachedBuffer &&buffer,
                            aos::monotonic_clock::time_point now) {
    QueueSizedFlatbuffer(std::move(buffer));
    FlushAtThreshold(now);
  }
  // Unconditionally queues the buffer.
  void QueueSizedFlatbuffer(flatbuffers::DetachedBuffer &&buffer) {
    if (ran_out_of_space_) {
      return;
    }
    encoder_->Encode(std::move(buffer));
  }

  // Queues up data in span. May copy or may write it to disk immediately.
  void QueueSpan(absl::Span<const uint8_t> span);

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

  // The maximum time for a single write call, or 0 if none have been performed.
  std::chrono::nanoseconds max_write_time() const { return max_write_time_; }
  // The number of bytes in the longest write call, or -1 if none have been
  // performed.
  int max_write_time_bytes() const { return max_write_time_bytes_; }
  // The number of buffers in the longest write call, or -1 if none have been
  // performed.
  int max_write_time_messages() const { return max_write_time_messages_; }
  // The total time spent in write calls.
  std::chrono::nanoseconds total_write_time() const {
    return total_write_time_;
  }
  // The total number of writes which have been performed.
  int total_write_count() const { return total_write_count_; }
  // The total number of messages which have been written.
  int total_write_messages() const { return total_write_messages_; }
  // The total number of bytes which have been written.
  int total_write_bytes() const { return total_write_bytes_; }
  void ResetStatistics() {
    max_write_time_ = std::chrono::nanoseconds::zero();
    max_write_time_bytes_ = -1;
    max_write_time_messages_ = -1;
    total_write_time_ = std::chrono::nanoseconds::zero();
    total_write_count_ = 0;
    total_write_messages_ = 0;
    total_write_bytes_ = 0;
  }

 private:
  // Performs a single writev call with as much of the data we have queued up as
  // possible.
  //
  // This will normally take all of the data we have queued up, unless an
  // encoder has spit out a big enough chunk all at once that we can't manage
  // all of it.
  void Flush();

  // write_return is what write(2) or writev(2) returned. write_size is the
  // number of bytes we expected it to write.
  void HandleWriteReturn(ssize_t write_return, size_t write_size);

  void UpdateStatsForWrite(aos::monotonic_clock::duration duration,
                           ssize_t written, int iovec_size);

  // Flushes data if we've reached the threshold to do that as part of normal
  // operation either due to the outstanding queued data, or because we have
  // passed our flush period.  now is the current time to save some CPU grabbing
  // the current time.  It just needs to be close.
  void FlushAtThreshold(aos::monotonic_clock::time_point now);

  std::string filename_;
  std::unique_ptr<DetachedBufferEncoder> encoder_;

  int fd_ = -1;
  bool ran_out_of_space_ = false;
  bool acknowledge_ran_out_of_space_ = false;

  // List of iovecs to use with writev.  This is a member variable to avoid
  // churn.
  std::vector<struct iovec> iovec_;

  std::chrono::nanoseconds max_write_time_ = std::chrono::nanoseconds::zero();
  int max_write_time_bytes_ = -1;
  int max_write_time_messages_ = -1;
  std::chrono::nanoseconds total_write_time_ = std::chrono::nanoseconds::zero();
  int total_write_count_ = 0;
  int total_write_messages_ = 0;
  int total_write_bytes_ = 0;

  aos::monotonic_clock::time_point last_flush_time_ =
      aos::monotonic_clock::min_time;
};

// Packes a message pointed to by the context into a MessageHeader.
flatbuffers::Offset<MessageHeader> PackMessage(
    flatbuffers::FlatBufferBuilder *fbb, const Context &context,
    int channel_index, LogType log_type);

// Reads the last header from a log file.  This handles any duplicate headers
// that were written.
std::optional<SizePrefixedFlatbufferVector<LogFileHeader>> ReadHeader(
    std::string_view filename);
// Reads the Nth message from a log file, excluding the header.  Note: this
// doesn't handle duplicate headers.
std::optional<SizePrefixedFlatbufferVector<MessageHeader>> ReadNthMessage(
    std::string_view filename, size_t n);

// Class to read chunks out of a log file.
class SpanReader {
 public:
  SpanReader(std::string_view filename);

  std::string_view filename() const { return filename_; }

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
};

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
  std::optional<SizePrefixedFlatbufferVector<MessageHeader>> ReadMessage();

  // The time at which we need to read another chunk from the logfile.
  monotonic_clock::time_point queue_data_time() const {
    return newest_timestamp() - max_out_of_order_duration();
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
  std::optional<SizePrefixedFlatbufferVector<MessageHeader>> ReadMessage();

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

  // The data (either a timestamp header, or a data header).
  SizePrefixedFlatbufferVector<MessageHeader> data;

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

  SizePrefixedFlatbufferVector<MessageHeader> data;
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

  // Sets a callback to be called whenever a full message is queued.
  void set_timestamp_callback(std::function<void(TimestampedMessage *)> fn) {
    timestamp_callback_ = fn;
  }

 private:
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

  // Queues up data until we have at least one message >= to time t.
  // Useful for triggering a remote node to read enough data to have the
  // timestamp you care about available.
  void QueueUnmatchedUntil(BootTimestamp t);

  // Queues m into matched_messages_.
  void QueueMessage(Message *m);

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
  // Time this node is queued up until.  Used for caching.
  BootTimestamp queued_until_ = BootTimestamp::min_time();

  std::function<void(TimestampedMessage *)> timestamp_callback_;
};

// Returns the node name with a trailing space, or an empty string if we are on
// a single node.
std::string MaybeNodeName(const Node *);

}  // namespace aos::logger

#endif  // AOS_EVENTS_LOGGING_LOGFILE_UTILS_H_
