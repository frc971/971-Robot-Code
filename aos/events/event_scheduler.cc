#include "aos/events/event_scheduler.h"

#include <algorithm>
#include <deque>

#include "aos/events/event_loop.h"
#include "aos/logging/implementations.h"

namespace aos {

EventScheduler::Token EventScheduler::Schedule(
    distributed_clock::time_point time, ::std::function<void()> callback) {
  return events_list_.emplace(time, callback);
}

void EventScheduler::Deschedule(EventScheduler::Token token) {
  events_list_.erase(token);
}

void EventScheduler::RunFor(distributed_clock::duration duration) {
  const distributed_clock::time_point end_time =
      distributed_now() + duration;
  logging::ScopedLogRestorer prev_logger;
  is_running_ = true;
  for (std::function<void()> &on_run : on_run_) {
    on_run();
  }
  on_run_.clear();
  while (!events_list_.empty() && is_running_) {
    auto iter = events_list_.begin();
    distributed_clock::time_point next_time = iter->first;
    if (next_time > end_time) {
      break;
    }
    now_ = iter->first;
    ::std::function<void()> callback = ::std::move(iter->second);
    events_list_.erase(iter);
    callback();
  }
  now_ = end_time;
}

void EventScheduler::Run() {
  logging::ScopedLogRestorer prev_logger;
  is_running_ = true;
  for (std::function<void()> &on_run : on_run_) {
    on_run();
  }
  on_run_.clear();
  while (!events_list_.empty() && is_running_) {
    auto iter = events_list_.begin();
    now_ = iter->first;
    ::std::function<void()> callback = ::std::move(iter->second);
    events_list_.erase(iter);
    callback();
  }
}

std::ostream &operator<<(std::ostream &stream,
                         const aos::distributed_clock::time_point &now) {
  // Print it the same way we print a monotonic time.  Literally.
  stream << monotonic_clock::time_point(now.time_since_epoch());
  return stream;
}

}  // namespace aos
