#ifndef AOS_EVENTS_LOGGING_LOGFILE_UTILS_H_
#define AOS_EVENTS_LOGGING_LOGFILE_UTILS_H_

#include <sys/uio.h>

#include <deque>
#include <optional>
#include <string_view>
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
  kLogMessageAndDeliveryTime
};


// This class manages efficiently writing a sequence of detached buffers to a
// file.  It queues them up and batches the write operation.
class DetachedBufferWriter {
 public:
  DetachedBufferWriter(std::string_view filename);
  ~DetachedBufferWriter();

  // TODO(austin): Snappy compress the log file if it ends with .snappy!

  // Queues up a finished FlatBufferBuilder to be written.  Steals the detached
  // buffer from it.
  void QueueSizedFlatbuffer(flatbuffers::FlatBufferBuilder *fbb);
  // Queues up a detached buffer directly.
  void QueueSizedFlatbuffer(flatbuffers::DetachedBuffer &&buffer);

  // Triggers data to be provided to the kernel and written.
  void Flush();

 private:
  int fd_ = -1;

  // Size of all the data in the queue.
  size_t queued_size_ = 0;

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

// Class to read chunks out of a log file.
class SpanReader {
 public:
  SpanReader(std::string_view filename);

  ~SpanReader() { close(fd_); }

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

  // Returns the header from the log file.
  const LogFileHeader *log_file_header() const {
    return flatbuffers::GetSizePrefixedRoot<LogFileHeader>(
        configuration_.data());
  }

  // Returns the minimum maount of data needed to queue up for sorting before
  // ware guarenteed to not see data out of order.
  std::chrono::nanoseconds max_out_of_order_duration() const {
    return max_out_of_order_duration_;
  }

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

  // Vector holding the data for the configuration.
  std::vector<uint8_t> configuration_;

  // Minimum amount of data to queue up for sorting before we are guarenteed
  // to not see data out of order.
  std::chrono::nanoseconds max_out_of_order_duration_;

  // Timestamp of the newest message in a channel queue.
  monotonic_clock::time_point newest_timestamp_ = monotonic_clock::min_time;
};

// We need to read a large chunk at a time, then kit it up into parts and
// sort.
//
// We want to read 256 KB chunks at a time.  This is the fastest read size.
// This leaves us with a fragmentation problem though.
//
// The easy answer is to read 256 KB chunks.  Then, malloc and memcpy those
// chunks into single flatbuffer messages and manage them in a sorted queue.
// Everything is copied three times (into 256 kb buffer, then into separate
// buffer, then into sender), but none of it is all that expensive.  We can
// optimize if it is slow later.
//
// As we place the elements in the sorted list of times, keep doing this
// until we read a message that is newer than the threshold.
//
// Then repeat.  Keep filling up the sorted list with 256 KB chunks (need a
// small state machine so we can resume), and keep pulling messages back out
// and sending.
//
// For sorting, we want to use the fact that each channel is sorted, and
// then merge sort the channels.  Have a vector of deques, and then hold a
// sorted list of pointers to those.
class SortedMessageReader {
 public:
  SortedMessageReader(std::string_view filename);

  // Returns the header from the log file.
  const LogFileHeader *log_file_header() const {
    return message_reader_.log_file_header();
  }

  // Returns a pointer to the channel with the oldest message in it, and the
  // timestamp.
  const std::pair<monotonic_clock::time_point, int> &oldest_message() const {
    return channel_heap_.front();
  }

  // Returns the number of channels with data still in them.
  size_t active_channel_count() const { return channel_heap_.size(); }

  // Returns the configuration from the log file header.
  const Configuration *configuration() const {
    return log_file_header()->configuration();
  }

  // Returns the start time on both the monotonic and realtime clocks.
  monotonic_clock::time_point monotonic_start_time() {
    return monotonic_clock::time_point(
        std::chrono::nanoseconds(log_file_header()->monotonic_start_time()));
  }
  realtime_clock::time_point realtime_start_time() {
    return realtime_clock::time_point(
        std::chrono::nanoseconds(log_file_header()->realtime_start_time()));
  }

  // Returns the node who's point of view this log file is from.  Make sure this
  // is a pointer in the configuration() nodes list so it can be consumed
  // elsewhere.
  const Node *node() const {
    if (configuration()->has_nodes()) {
      CHECK(log_file_header()->has_node());
      CHECK(log_file_header()->node()->has_name());
      return configuration::GetNode(
          configuration(), log_file_header()->node()->name()->string_view());
    } else {
      CHECK(!log_file_header()->has_node());
      return nullptr;
    }
  }

  // Pops a pointer to the channel with the oldest message in it, and the
  // timestamp.
  std::tuple<monotonic_clock::time_point, int, FlatbufferVector<MessageHeader>>
  PopOldestChannel();

 private:
  // Adds more messages to the sorted list.
  void QueueMessages();

  // Moves the message to the correct channel queue.
  void EmplaceDataBack(FlatbufferVector<MessageHeader> &&new_data);

  // Pushes a pointer to the channel for the given timestamp to the sorted
  // channel list.
  void PushChannelHeap(monotonic_clock::time_point timestamp,
                       int channel_index);


  // Datastructure to hold the list of messages, cached timestamp for the
  // oldest message, and sender to send with.
  struct ChannelData {
    monotonic_clock::time_point oldest_timestamp = monotonic_clock::min_time;
    std::deque<FlatbufferVector<MessageHeader>> data;
    std::unique_ptr<RawSender> raw_sender;

    // Returns the oldest message.
    const FlatbufferVector<MessageHeader> &front() { return data.front(); }

    // Returns the timestamp for the oldest message.
    const monotonic_clock::time_point front_timestamp() {
      return monotonic_clock::time_point(
          std::chrono::nanoseconds(front().message().monotonic_sent_time()));
    }
  };

  MessageReader message_reader_;

  // TODO(austin): Multithreaded read at some point.  Gotta go faster!
  // Especially if we start compressing.

  // List of channels and messages for them.
  std::vector<ChannelData> channels_;

  // Heap of channels so we can track which channel to send next.
  std::vector<std::pair<monotonic_clock::time_point, int>> channel_heap_;

};

}  // namespace logger
}  // namespace aos

#endif  // AOS_EVENTS_LOGGING_LOGFILE_UTILS_H_
