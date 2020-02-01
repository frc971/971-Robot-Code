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

// This clock is the basis for distributed time.  It is used to synchronize time
// between multiple nodes.  This is a new type so conversions to and from the
// monotonic and realtime clocks aren't implicit.
class distributed_clock {
 public:
  typedef ::std::chrono::nanoseconds::rep rep;
  typedef ::std::chrono::nanoseconds::period period;
  typedef ::std::chrono::nanoseconds duration;
  typedef ::std::chrono::time_point<distributed_clock> time_point;

  // This clock is the base clock for the simulation and everything is synced to
  // it.  It never jumps.
  static constexpr bool is_steady = true;

  // Returns the epoch (0).
  static constexpr time_point epoch() { return time_point(zero()); }

  static constexpr duration zero() { return duration(0); }

  static constexpr time_point min_time{
      time_point(duration(::std::numeric_limits<duration::rep>::min()))};
  static constexpr time_point max_time{
      time_point(duration(::std::numeric_limits<duration::rep>::max()))};
};

std::ostream &operator<<(std::ostream &stream,
                         const aos::distributed_clock::time_point &now);

class EventScheduler {
 public:
  using ChannelType =
      std::multimap<distributed_clock::time_point, std::function<void()>>;
  using Token = ChannelType::iterator;

  // Schedule an event with a callback function
  // Returns an iterator to the event
  Token Schedule(distributed_clock::time_point time,
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
  void RunFor(distributed_clock::duration duration);

  void Exit() { is_running_ = false; }

  bool is_running() const { return is_running_; }

  distributed_clock::time_point distributed_now() const { return now_; }

 private:
  // Current execution time.
  distributed_clock::time_point now_ = distributed_clock::epoch();

  std::vector<std::function<void()>> on_run_;

  // Multimap holding times to run functions.  These are stored in order, and
  // the order is the callback tree.
  ChannelType events_list_;
  bool is_running_ = false;
};

}  // namespace aos

#endif  // AOS_EVENTS_EVENT_SCHEDULER_H_
