#ifndef AOS_EVENTS_EVENT_SCHEDULER_H_
#define AOS_EVENTS_EVENT_SCHEDULER_H_

#include <algorithm>
#include <map>
#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include "aos/events/event_loop.h"
#include "aos/time/time.h"
#include "glog/logging.h"

namespace aos {

class EventScheduler {
 public:
  using ChannelType =
      std::multimap<monotonic_clock::time_point, std::function<void()>>;
  using Token = ChannelType::iterator;

  // Schedule an event with a callback function
  // Returns an iterator to the event
  Token Schedule(monotonic_clock::time_point time,
                 std::function<void()> callback);

  // Schedules a callback when the event scheduler starts.
  void ScheduleOnRun(std::function<void()> callback) {
    on_run_.emplace_back(std::move(callback));
  }

  Token InvalidToken() { return events_list_.end(); }

  // Deschedule an event by its iterator
  void Deschedule(Token token);

  // Runs until exited.
  void Run();
  // Runs for a duration.
  void RunFor(monotonic_clock::duration duration);

  void Exit() { is_running_ = false; }

  bool is_running() const { return is_running_; }

  monotonic_clock::time_point monotonic_now() const { return now_; }

  realtime_clock::time_point realtime_now() const {
    return realtime_clock::time_point(monotonic_now().time_since_epoch() +
                                      realtime_offset_);
  }

  // Sets realtime clock to realtime_now for a given monotonic clock.
  void SetRealtimeOffset(monotonic_clock::time_point monotonic_now,
                         realtime_clock::time_point realtime_now) {
    realtime_offset_ =
        realtime_now.time_since_epoch() - monotonic_now.time_since_epoch();
  }

 private:
  // Current execution time.
  monotonic_clock::time_point now_ = monotonic_clock::epoch();
  std::chrono::nanoseconds realtime_offset_ = std::chrono::seconds(0);

  std::vector<std::function<void()>> on_run_;

  // Multimap holding times to run functions.  These are stored in order, and
  // the order is the callback tree.
  ChannelType events_list_;
  bool is_running_ = false;
};

}  // namespace aos

#endif  // AOS_EVENTS_EVENT_SCHEDULER_H_
