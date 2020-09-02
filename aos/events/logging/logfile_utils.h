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

#include "absl/types/span.h"
#include "aos/events/event_loop.h"
#include "aos/events/logging/logger_generated.h"
#include "flatbuffers/flatbuffers.h"

namespace aos {
namespace logger {

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
// file.  It queues them up and batches the write operation.
class DetachedBufferWriter {
 public:
  DetachedBufferWriter(std::string_view filename);
  DetachedBufferWriter(DetachedBufferWriter &&other);
  DetachedBufferWriter(const DetachedBufferWriter &) = delete;

  ~DetachedBufferWriter();

  DetachedBufferWriter &operator=(DetachedBufferWriter &&other);
  DetachedBufferWriter &operator=(const DetachedBufferWriter &) = delete;

  std::string_view filename() const { return filename_; }

  // Rewrites a location in a file (relative to the start) to have new data in
  // it.  The main use case is updating start times after a log file starts.
  void RewriteLocation(off64_t offset, absl::Span<const uint8_t> data);

  // TODO(austin): Snappy compress the log file if it ends with .snappy!

  // Queues up a finished FlatBufferBuilder to be written.  Steals the detached
  // buffer from it.
  void QueueSizedFlatbuffer(flatbuffers::FlatBufferBuilder *fbb);
  // Queues up a detached buffer directly.
  void QueueSizedFlatbuffer(flatbuffers::DetachedBuffer &&buffer);
  // Writes a Span.  This is not terribly optimized right now.
  void WriteSizedFlatbuffer(absl::Span<const uint8_t> span);

  // Triggers data to be provided to the kernel and written.
  void Flush();

  // Returns the number of bytes written.
  size_t written_size() const { return written_size_; }

  // Returns the number of bytes written or currently queued.
  size_t total_size() const { return written_size_ + queued_size_; }

 private:
  std::string filename_;

  int fd_ = -1;

  // Size of all the data in the queue.
  size_t queued_size_ = 0;
  size_t written_size_ = 0;

  // List of buffers to flush.
  std::vector<flatbuffers::DetachedBuffer> queue_;
  // List of iovecs to use with writev.  This is a member variable to avoid
  // churn.
  std::vector<struct iovec> iovec_;
};

// Packes a message pointed to by the context into a MessageHeader.
flatbuffers::Offset<MessageHeader> PackMessage(
    flatbuffers::FlatBufferBuilder *fbb, const Context &context,
    int channel_index, LogType log_type);

FlatbufferVector<LogFileHeader> ReadHeader(std::string_view filename);

// Class to read chunks out of a log file.
class SpanReader {
 public:
  SpanReader(std::string_view filename);

  ~SpanReader() { close(fd_); }

  std::string_view filename() const { return filename_; }

  // Returns a span with the data for a message from the log file, excluding
  // the size.
  absl::Span<const uint8_t> ReadMessage();

  // Returns true if there is a full message available in the buffer, or if we
  // will have to read more data from disk.
  bool MessageAvailable();

 private:
  // TODO(austin): Optimization:
  //   Allocate the 256k blocks like we do today.  But, refcount them with
  //   shared_ptr pointed to by the messageheader that is returned.  This avoids
  //   the copy.  Need to do more benchmarking.

  // Reads a chunk of data into data_.  Returns false if no data was read.
  bool ReadBlock();

  const std::string filename_;

  // File descriptor for the log file.
  int fd_ = -1;

  // Allocator which doesn't zero initialize memory.
  template <typename T>
  struct DefaultInitAllocator {
    typedef T value_type;

    template <typename U>
    void construct(U *p) {
      ::new (static_cast<void *>(p)) U;
    }

    template <typename U, typename... Args>
    void construct(U *p, Args &&... args) {
      ::new (static_cast<void *>(p)) U(std::forward<Args>(args)...);
    }

    T *allocate(std::size_t n) {
      return reinterpret_cast<T *>(::operator new(sizeof(T) * n));
    }

    template <typename U>
    void deallocate(U *p, std::size_t /*n*/) {
      ::operator delete(static_cast<void *>(p));
    }
  };

  // Vector to read into.  This uses an allocator which doesn't zero
  // initialize the memory.
  std::vector<uint8_t, DefaultInitAllocator<uint8_t>> data_;

  // Amount of data consumed already in data_.
  size_t consumed_data_ = 0;

  // Cached bit for if we have reached the end of the file.  Otherwise we will
  // hammer on the kernel asking for more data each time we send.
  bool end_of_file_ = false;
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
  const FlatbufferVector<LogFileHeader> &raw_log_file_header() const {
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
  std::optional<FlatbufferVector<MessageHeader>> ReadMessage();

  // The time at which we need to read another chunk from the logfile.
  monotonic_clock::time_point queue_data_time() const {
    return newest_timestamp() - max_out_of_order_duration();
  }

 private:
  // Log chunk reader.
  SpanReader span_reader_;

  // Vector holding the raw data for the log file header.
  FlatbufferVector<LogFileHeader> raw_log_file_header_;

  // Minimum amount of data to queue up for sorting before we are guarenteed
  // to not see data out of order.
  std::chrono::nanoseconds max_out_of_order_duration_;

  // Timestamp of the newest message in a channel queue.
  monotonic_clock::time_point newest_timestamp_ = monotonic_clock::min_time;
};

class TimestampMerger;

// A design requirement is that the relevant data for a channel is not more than
// max_out_of_order_duration out of order. We approach sorting in layers.
//
// 1) Split each (maybe chunked) log file into one queue per channel.  Read this
//    log file looking for data pertaining to a specific node.
//    (SplitMessageReader)
// 2) Merge all the data per channel from the different log files into a sorted
//    list of timestamps and messages. (TimestampMerger)
// 3) Combine the timestamps and messages. (TimestampMerger)
// 4) Merge all the channels to produce the next message on a node.
//    (ChannelMerger)
// 5) Duplicate this entire stack per node.

// This class splits messages and timestamps up into a queue per channel, and
// handles reading data from multiple chunks.
class SplitMessageReader {
 public:
  SplitMessageReader(const std::vector<std::string> &filenames);

  // Sets the TimestampMerger that gets notified for each channel.  The node
  // that the TimestampMerger is merging as needs to be passed in.
  void SetTimestampMerger(TimestampMerger *timestamp_merger, int channel,
                          const Node *target_node);

  // Returns the (timestamp, queue_index, message_header) for the oldest message
  // in a channel, or max_time if there is nothing in the channel.
  std::tuple<monotonic_clock::time_point, uint32_t, const MessageHeader *>
  oldest_message(int channel) {
    return channels_[channel].data.front_timestamp();
  }

  // Returns the (timestamp, queue_index, message_header) for the oldest
  // delivery time in a channel, or max_time if there is nothing in the channel.
  std::tuple<monotonic_clock::time_point, uint32_t, const MessageHeader *>
  oldest_message(int channel, int destination_node) {
    return channels_[channel].timestamps[destination_node].front_timestamp();
  }

  // Returns the timestamp, queue_index, and message for the oldest data on a
  // channel.  Requeues data as needed.
  std::tuple<monotonic_clock::time_point, uint32_t,
             FlatbufferVector<MessageHeader>>
  PopOldest(int channel_index);

  // Returns the timestamp, queue_index, and message for the oldest timestamp on
  // a channel delivered to a node.  Requeues data as needed.
  std::tuple<monotonic_clock::time_point, uint32_t,
             FlatbufferVector<MessageHeader>>
  PopOldestTimestamp(int channel, int node_index);

  // Returns the header for the log files.
  const LogFileHeader *log_file_header() const {
    return &log_file_header_.message();
  }

  const FlatbufferVector<LogFileHeader> &raw_log_file_header() const {
    return log_file_header_;
  }

  // Returns the starting time for this set of log files.
  monotonic_clock::time_point monotonic_start_time() {
    return monotonic_clock::time_point(
        std::chrono::nanoseconds(log_file_header()->monotonic_start_time()));
  }
  realtime_clock::time_point realtime_start_time() {
    return realtime_clock::time_point(
        std::chrono::nanoseconds(log_file_header()->realtime_start_time()));
  }

  // Returns the configuration from the log file header.
  const Configuration *configuration() const {
    return log_file_header()->configuration();
  }

  // Returns the node who's point of view this log file is from.  Make sure this
  // is a pointer in the configuration() nodes list so it can be consumed
  // elsewhere.
  const Node *node() const {
    if (configuration()->has_nodes()) {
      return configuration::GetNodeOrDie(configuration(),
                                         log_file_header()->node());
    } else {
      CHECK(!log_file_header()->has_node());
      return nullptr;
    }
  }

  // Returns the timestamp of the newest message read from the log file, and the
  // timestamp that we need to re-queue data.
  monotonic_clock::time_point newest_timestamp() const {
    return newest_timestamp_;
  }

  // Returns the next time to trigger a requeue.
  monotonic_clock::time_point time_to_queue() const { return time_to_queue_; }

  // Returns the minimum amount of data needed to queue up for sorting before
  // ware guarenteed to not see data out of order.
  std::chrono::nanoseconds max_out_of_order_duration() const {
    return message_reader_->max_out_of_order_duration();
  }

  std::string_view filename() const { return message_reader_->filename(); }

  // Adds more messages to the sorted list.  This reads enough data such that
  // oldest_message_time can be replayed safely.  Returns false if the log file
  // has all been read.
  bool QueueMessages(monotonic_clock::time_point oldest_message_time);

  // Returns debug strings for a channel, and timestamps for a node.
  std::string DebugString(int channel) const;
  std::string DebugString(int channel, int node_index) const;

  // Returns true if all the messages have been queued from the last log file in
  // the list of log files chunks.
  bool at_end() const { return at_end_; }

 private:
  // TODO(austin): Need to copy or refcount the message instead of running
  // multiple copies of the reader.  Or maybe have a "as_node" index and hide it
  // inside.

  // Moves to the next log file in the list.
  bool NextLogFile();

  // Filenames of the log files.
  std::vector<std::string> filenames_;
  // And the index of the next file to open.
  size_t next_filename_index_ = 0;

  // Node we are reading as.
  const Node *target_node_ = nullptr;

  // Log file header to report.  This is a copy.
  FlatbufferVector<LogFileHeader> log_file_header_;
  // Current log file being read.
  std::unique_ptr<MessageReader> message_reader_;

  // Datastructure to hold the list of messages, cached timestamp for the
  // oldest message, and sender to send with.
  struct MessageHeaderQueue {
    // If true, this is a timestamp queue.
    bool timestamps = false;

    // Returns a reference to the the oldest message.
    FlatbufferVector<MessageHeader> &front() {
      CHECK_GT(data_.size(), 0u);
      return data_.front();
    }

    // Adds a message to the back of the queue. Returns true if it was actually
    // emplaced.
    bool emplace_back(FlatbufferVector<MessageHeader> &&msg);

    // Drops the front message.  Invalidates the front() reference.
    void PopFront();

    // The size of the queue.
    size_t size() { return data_.size(); }

    // Returns a debug string with info about each message in the queue.
    std::string DebugString() const;

    // Returns the (timestamp, queue_index, message_header) for the oldest
    // message.
    const std::tuple<monotonic_clock::time_point, uint32_t,
                     const MessageHeader *>
    front_timestamp() {
      const MessageHeader &message = front().message();
      return std::make_tuple(
          monotonic_clock::time_point(
              std::chrono::nanoseconds(message.monotonic_sent_time())),
          message.queue_index(), &message);
    }

    // Pointer to the timestamp merger for this queue if available.
    TimestampMerger *timestamp_merger = nullptr;
    // Pointer to the reader which feeds this queue.
    SplitMessageReader *split_reader = nullptr;

   private:
    // The data.
    std::deque<FlatbufferVector<MessageHeader>> data_;
  };

  // All the queues needed for a channel.  There isn't going to be data in all
  // of these.
  struct ChannelData {
    // The data queue for the channel.
    MessageHeaderQueue data;
    // Queues for timestamps for each node.
    std::vector<MessageHeaderQueue> timestamps;
  };

  // Data for all the channels.
  std::vector<ChannelData> channels_;

  // Once we know the node that this SplitMessageReader will be writing as,
  // there will be only one MessageHeaderQueue that a specific channel matches.
  // Precompute this here for efficiency.
  std::vector<MessageHeaderQueue *> channels_to_write_;

  monotonic_clock::time_point time_to_queue_ = monotonic_clock::min_time;

  // Latches true when we hit the end of the last log file and there is no sense
  // poking it further.
  bool at_end_ = false;

  // Timestamp of the newest message that was read and actually queued.  We want
  // to track this independently from the log file because we need the
  // timestamps here to be timestamps of messages that are queued.
  monotonic_clock::time_point newest_timestamp_ = monotonic_clock::min_time;
};

class ChannelMerger;

// Sorts channels (and timestamps) from multiple log files for a single channel.
class TimestampMerger {
 public:
  TimestampMerger(const Configuration *configuration,
                  std::vector<SplitMessageReader *> split_message_readers,
                  int channel_index, const Node *target_node,
                  ChannelMerger *channel_merger);

  // Metadata used to schedule the message.
  struct DeliveryTimestamp {
    monotonic_clock::time_point monotonic_event_time =
        monotonic_clock::min_time;
    realtime_clock::time_point realtime_event_time = realtime_clock::min_time;

    monotonic_clock::time_point monotonic_remote_time =
        monotonic_clock::min_time;
    realtime_clock::time_point realtime_remote_time = realtime_clock::min_time;
    uint32_t remote_queue_index = 0xffffffff;
  };

  // Pushes SplitMessageReader onto the timestamp heap.  This should only be
  // called when timestamps are placed in the channel this class is merging for
  // the reader.
  void UpdateTimestamp(
      SplitMessageReader *split_message_reader,
      std::tuple<monotonic_clock::time_point, uint32_t, const MessageHeader *>
          oldest_message_time) {
    PushTimestampHeap(oldest_message_time, split_message_reader);
  }
  // Pushes SplitMessageReader onto the message heap.  This should only be
  // called when data is placed in the channel this class is merging for the
  // reader.
  void Update(
      SplitMessageReader *split_message_reader,
      std::tuple<monotonic_clock::time_point, uint32_t, const MessageHeader *>
          oldest_message_time) {
    PushMessageHeap(oldest_message_time, split_message_reader);
  }

  // Returns the oldest combined timestamp and data for this channel.  If there
  // isn't a matching piece of data, returns only the timestamp with no data.
  // The caller can determine what the appropriate action is to recover.
  std::tuple<DeliveryTimestamp, FlatbufferVector<MessageHeader>> PopOldest();

  // Tracks if the channel merger has pushed this onto it's heap or not.
  bool pushed() { return pushed_; }
  // Sets if this has been pushed to the channel merger heap.  Should only be
  // called by the channel merger.
  void set_pushed(bool pushed) { pushed_ = pushed; }

  // Returns a debug string with the heaps printed out.
  std::string DebugString() const;

  // Returns true if we have timestamps.
  bool has_timestamps() const { return has_timestamps_; }

  // Records that one of the log files ran out of data.  This should only be
  // called by a SplitMessageReader.
  void NoticeAtEnd();

  aos::monotonic_clock::time_point channel_merger_time() {
    if (has_timestamps_) {
      return std::get<0>(timestamp_heap_[0]);
    } else {
      return std::get<0>(message_heap_[0]);
    }
  }

 private:
  // Pushes messages and timestamps to the corresponding heaps.
  void PushMessageHeap(
      std::tuple<monotonic_clock::time_point, uint32_t, const MessageHeader *>
          timestamp,
      SplitMessageReader *split_message_reader);
  void PushTimestampHeap(
      std::tuple<monotonic_clock::time_point, uint32_t, const MessageHeader *>
          timestamp,
      SplitMessageReader *split_message_reader);

  // Pops a message from the message heap.  This automatically triggers the
  // split message reader to re-fetch any new data.
  std::tuple<monotonic_clock::time_point, uint32_t,
             FlatbufferVector<MessageHeader>>
  PopMessageHeap();

  std::tuple<monotonic_clock::time_point, uint32_t, const MessageHeader *>
  oldest_message() const;
  std::tuple<monotonic_clock::time_point, uint32_t, const MessageHeader *>
  oldest_timestamp() const;
  // Pops a message from the timestamp heap.  This automatically triggers the
  // split message reader to re-fetch any new data.
  std::tuple<monotonic_clock::time_point, uint32_t,
             FlatbufferVector<MessageHeader>>
  PopTimestampHeap();

  const Configuration *configuration_;

  // If true, this is a forwarded channel and timestamps should be matched.
  bool has_timestamps_ = false;

  // Tracks if the ChannelMerger has pushed this onto it's queue.
  bool pushed_ = false;

  // The split message readers used for source data.
  std::vector<SplitMessageReader *> split_message_readers_;

  // The channel to merge.
  int channel_index_;

  // Our node.
  int node_index_;

  // Heaps for messages and timestamps.
  std::vector<
      std::tuple<monotonic_clock::time_point, uint32_t, SplitMessageReader *>>
      message_heap_;
  std::vector<
      std::tuple<monotonic_clock::time_point, uint32_t, SplitMessageReader *>>
      timestamp_heap_;

  // Parent channel merger.
  ChannelMerger *channel_merger_;
};

// This class handles constructing all the split message readers, channel
// mergers, and combining the results.
class ChannelMerger {
 public:
  // Builds a ChannelMerger around a set of log files.  These are of the format:
  //   {
  //     {log1_part0, log1_part1, ...},
  //     {log2}
  //   }
  // The inner vector is a list of log file chunks which form up a log file.
  // The outer vector is a list of log files with subsets of the messages, or
  // messages from different nodes.
  ChannelMerger(const std::vector<std::vector<std::string>> &filenames);

  // Returns the nodes that we know how to merge.
  const std::vector<const Node *> nodes() const;
  // Sets the node that we will return messages as.  Returns true if the node
  // has log files and will produce data.  This can only be called once, and
  // will likely corrupt state if called a second time.
  bool SetNode(const Node *target_node);

  // Everything else needs the node set before it works.

  // Returns a timestamp for the oldest message in this group of logfiles.
  monotonic_clock::time_point OldestMessageTime() const;
  // Pops the oldest message.
  std::tuple<TimestampMerger::DeliveryTimestamp, int,
             FlatbufferVector<MessageHeader>>
  PopOldest();

  // Returns the config for this set of log files.
  const Configuration *configuration() const {
    return log_file_header()->configuration();
  }

  const LogFileHeader *log_file_header() const {
    return &log_file_header_.message();
  }

  // Returns the start times for the configured node's log files.
  monotonic_clock::time_point monotonic_start_time() const {
    return monotonic_clock::time_point(
        std::chrono::nanoseconds(log_file_header()->monotonic_start_time()));
  }
  realtime_clock::time_point realtime_start_time() const {
    return realtime_clock::time_point(
        std::chrono::nanoseconds(log_file_header()->realtime_start_time()));
  }

  // Returns the node set by SetNode above.
  const Node *node() const { return node_; }

  // Called by the TimestampMerger when new data is available with the provided
  // timestamp and channel_index.
  void Update(monotonic_clock::time_point timestamp, int channel_index) {
    PushChannelHeap(timestamp, channel_index);
  }

  // Returns a debug string with all the heaps in it.  Generally only useful for
  // debugging what went wrong.
  std::string DebugString() const;

  // Returns true if one of the log files has finished reading everything.  When
  // log file chunks are involved, this means that the last chunk in a log file
  // has been read.  It is acceptable to be missing data at this point in time.
  bool at_end() const { return at_end_; }

  // Marks that one of the log files is at the end.  This should only be called
  // by timestamp mergers.
  void NoticeAtEnd() { at_end_ = true; }

 private:
  // Pushes the timestamp for new data on the provided channel.
  void PushChannelHeap(monotonic_clock::time_point timestamp,
                       int channel_index);

  // CHECKs that channel_heap_ and timestamp_heap_ are valid heaps.
  void VerifyHeaps();

  // All the message readers.
  std::vector<std::unique_ptr<SplitMessageReader>> split_message_readers_;

  // The log header we are claiming to be.
  FlatbufferVector<LogFileHeader> log_file_header_;

  // The timestamp mergers which combine data from the split message readers.
  std::vector<TimestampMerger> timestamp_mergers_;

  // A heap of the channel readers and timestamps for the oldest data in each.
  std::vector<std::pair<monotonic_clock::time_point, int>> channel_heap_;
  // A heap of just the timestamp channel readers and timestamps for the oldest
  // data in each.
  // TODO(austin): I think this is no longer used and can be removed (!)
  std::vector<std::pair<monotonic_clock::time_point, int>> timestamp_heap_;

  // Configured node.
  const Node *node_;

  bool at_end_ = false;

  // Cached copy of the list of nodes.
  std::vector<const Node *> nodes_;

  // Last time popped.  Used to detect events being returned out of order.
  monotonic_clock::time_point last_popped_time_ = monotonic_clock::min_time;
};

// Returns the node name with a trailing space, or an empty string if we are on
// a single node.
std::string MaybeNodeName(const Node *);

}  // namespace logger
}  // namespace aos

#endif  // AOS_EVENTS_LOGGING_LOGFILE_UTILS_H_
