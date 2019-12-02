#include "aos/events/event_scheduler.h"

#include <algorithm>
#include <deque>

#include "aos/events/event_loop.h"

namespace aos {

EventScheduler::Token EventScheduler::Schedule(
    ::aos::monotonic_clock::time_point time, ::std::function<void()> callback) {
  return events_list_.emplace(time, callback);
}

void EventScheduler::Deschedule(EventScheduler::Token token) {
  events_list_.erase(token);
}

void EventScheduler::RunFor(monotonic_clock::duration duration) {
  const ::aos::monotonic_clock::time_point end_time =
      monotonic_now() + duration;
  is_running_ = true;
  for (std::function<void()> &on_run : on_run_) {
    on_run();
  }
  on_run_.clear();
  while (!events_list_.empty() && is_running_) {
    auto iter = events_list_.begin();
    ::aos::monotonic_clock::time_point next_time = iter->first;
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

}  // namespace aos
