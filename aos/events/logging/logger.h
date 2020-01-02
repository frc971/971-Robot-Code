#ifndef AOS_EVENTS_LOGGER_H_
#define AOS_EVENTS_LOGGER_H_

#include <deque>
#include <vector>
#include <string_view>

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
  LogReader(std::string_view filename);
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
  // Queues at least max_out_of_order_duration_ messages into channels_.
  void QueueMessages();

  // Log chunk reader.
  SortedMessageReader sorted_message_reader_;

  std::vector<std::unique_ptr<RawSender>> channels_;

  SimulatedEventLoopFactory *event_loop_factory_ = nullptr;
  std::unique_ptr<EventLoop> event_loop_unique_ptr_;
  EventLoop *event_loop_ = nullptr;
  TimerHandler *timer_handler_;
};

}  // namespace logger
}  // namespace aos

#endif  // AOS_EVENTS_LOGGER_H_
