#ifndef AOS_EVENTS_LOGGER_H_
#define AOS_EVENTS_LOGGER_H_

#include <deque>
#include <vector>

#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "aos/events/event_loop.h"
#include "aos/events/logging/logfile_utils.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/time/time.h"
#include "flatbuffers/flatbuffers.h"

namespace aos {
namespace logger {

// Logs all channels available in the event loop to disk every 100 ms.
// Start by logging one message per channel to capture any state and
// configuration that is sent rately on a channel and would affect execution.
class Logger {
 public:
  Logger(DetachedBufferWriter *writer, EventLoop *event_loop,
         std::chrono::milliseconds polling_period =
             std::chrono::milliseconds(100));

 private:
  void DoLogData();

  EventLoop *event_loop_;
  DetachedBufferWriter *writer_;

  // Structure to track both a fetcher, and if the data fetched has been
  // written.  We may want to delay writing data to disk so that we don't let
  // data get too far out of order when written to disk so we can avoid making
  // it too hard to sort when reading.
  struct FetcherStruct {
    std::unique_ptr<RawFetcher> fetcher;
    bool written = false;

    LogType log_type;
  };

  std::vector<FetcherStruct> fetchers_;
  TimerHandler *timer_handler_;

  // Period to poll the channels.
  const std::chrono::milliseconds polling_period_;

  // Last time that data was written for all channels to disk.
  monotonic_clock::time_point last_synchronized_time_;

  // Max size that the header has consumed.  This much extra data will be
  // reserved in the builder to avoid reallocating.
  size_t max_header_size_ = 0;
};

// Replays all the channels in the logfile to the event loop.
class LogReader {
 public:
  LogReader(absl::string_view filename);
  ~LogReader();

  // Registers the timer and senders used to resend the messages from the log
  // file.
  void Register(EventLoop *event_loop);
  // Registers everything, but also updates the real time time in sync.  Runs
  // until the log file starts.
  void Register(SimulatedEventLoopFactory *factory);
  // Unregisters the senders.
  void Deregister();

  // TODO(austin): Remap channels?

  // Returns the configuration from the log file.
  const Configuration *configuration() const;

  // Returns the node that this log file was created on.
  const Node *node() const;

  // Returns the starting timestamp for the log file.
  monotonic_clock::time_point monotonic_start_time();
  realtime_clock::time_point realtime_start_time();

  // TODO(austin): Add the ability to re-publish the fetched messages.  Add 2
  // options, one which publishes them *now*, and another which publishes them
  // to the simulated event loop factory back in time where they actually
  // happened.

 private:
  // Reads a chunk of data into data_.  Returns false if no data was read.
  bool ReadBlock();

  // Returns true if there is a full message available in the buffer, or if we
  // will have to read more data from disk.
  bool MessageAvailable();

  // Returns a span with the data for a message from the log file, excluding
  // the size.
  absl::Span<const uint8_t> ReadMessage();

  // Queues at least max_out_of_order_duration_ messages into channels_.
  void QueueMessages();

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
  //
  // TODO(austin): Multithreaded read at some point.  Gotta go faster!
  // Especially if we start compressing.

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

  // Minimum amount of data to queue up for sorting before we are guarenteed
  // to not see data out of order.
  std::chrono::nanoseconds max_out_of_order_duration_;

  // File descriptor for the log file.
  int fd_ = -1;

  SimulatedEventLoopFactory *event_loop_factory_ = nullptr;
  std::unique_ptr<EventLoop> event_loop_unique_ptr_;
  EventLoop *event_loop_ = nullptr;
  TimerHandler *timer_handler_;

  // Vector to read into.  This uses an allocator which doesn't zero
  // initialize the memory.
  std::vector<uint8_t, DefaultInitAllocator<uint8_t>> data_;

  // Amount of data consumed already in data_.
  size_t consumed_data_ = 0;

  // Vector holding the data for the configuration.
  std::vector<uint8_t> configuration_;

  // Moves the message to the correct channel queue.
  void EmplaceDataBack(FlatbufferVector<MessageHeader> &&new_data);

  // Pushes a pointer to the channel for the given timestamp to the sorted
  // channel list.
  void PushChannelHeap(monotonic_clock::time_point timestamp,
                       int channel_index);

  // Returns a pointer to the channel with the oldest message in it, and the
  // timestamp.
  const std::pair<monotonic_clock::time_point, int> &oldest_message() const {
    return channel_heap_.front();
  }

  // Pops a pointer to the channel with the oldest message in it, and the
  // timestamp.
  std::pair<monotonic_clock::time_point, int> PopOldestChannel();

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

  // List of channels and messages for them.
  std::vector<ChannelData> channels_;

  // Heap of channels so we can track which channel to send next.
  std::vector<std::pair<monotonic_clock::time_point, int>> channel_heap_;

  // Timestamp of the newest message in a channel queue.
  monotonic_clock::time_point newest_timestamp_ = monotonic_clock::min_time;

  // The time at which we need to read another chunk from the logfile.
  monotonic_clock::time_point queue_data_time_ = monotonic_clock::min_time;

  // Cached bit for if we have reached the end of the file.  Otherwise we will
  // hammer on the kernel asking for more data each time we send.
  bool end_of_file_ = false;
};

}  // namespace logger
}  // namespace aos

#endif  // AOS_EVENTS_LOGGER_H_
