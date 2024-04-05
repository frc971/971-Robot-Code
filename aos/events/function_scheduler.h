#ifndef AOS_EVENTS_FUNCTION_SCHEDULER_H_
#define AOS_EVENTS_FUNCTION_SCHEDULER_H_

#include <functional>
#include <map>

#include "aos/events/event_loop.h"
#include "aos/time/time.h"

namespace aos {

// Simple class to call a function at a time with a timer.
class FunctionScheduler {
 public:
  FunctionScheduler(aos::EventLoop *event_loop);

  // Schedules the function to be run at the provided time.
  void ScheduleAt(std::function<void()> &&function,
                  aos::monotonic_clock::time_point time);

 private:
  void RunFunctions(aos::monotonic_clock::time_point now);

  aos::EventLoop *event_loop_;
  aos::TimerHandler *timer_;

  std::multimap<aos::monotonic_clock::time_point, std::function<void()>>
      functions_;
};

}  // namespace aos

#endif  // AOS_EVENTS_FUNCTION_SCHEDULER_H_
