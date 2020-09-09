#include "aos/events/event_scheduler.h"

#include <algorithm>
#include <deque>

#include "aos/events/event_loop.h"
#include "aos/logging/implementations.h"

namespace aos {

EventScheduler::Token EventScheduler::Schedule(
    monotonic_clock::time_point time, ::std::function<void()> callback) {
  return events_list_.emplace(time, callback);
}

void EventScheduler::Deschedule(EventScheduler::Token token) {
  // We basically want to DCHECK some nontrivial logic. Guard it with NDEBUG to ensure the compiler
  // realizes it's all unnecessary when not doing debug checks.
#ifndef NDEBUG
  {
    bool found = false;
    auto i = events_list_.begin();
    while (i != events_list_.end()) {
      if (i == token) {
        CHECK(!found) << ": The same iterator is in the multimap twice??";
        found = true;
      }
      ++i;
    }
    CHECK(found) << ": Trying to deschedule an event which is not scheduled";
  }
#endif
  events_list_.erase(token);
}

aos::monotonic_clock::time_point EventScheduler::OldestEvent() {
  if (events_list_.empty()) {
    return monotonic_clock::max_time;
  }

  return events_list_.begin()->first;
}

void EventScheduler::CallOldestEvent() {
  CHECK_GT(events_list_.size(), 0u);
  auto iter = events_list_.begin();
  monotonic_now_ = iter->first;
  monotonic_now_valid_ = true;

  ::std::function<void()> callback = ::std::move(iter->second);
  events_list_.erase(iter);
  callback();
  monotonic_now_valid_ = false;
}

void EventScheduler::RunOnRun() {
  for (std::function<void()> &on_run : on_run_) {
    on_run();
  }
  on_run_.clear();
}

std::ostream &operator<<(std::ostream &stream,
                         const aos::distributed_clock::time_point &now) {
  // Print it the same way we print a monotonic time.  Literally.
  stream << monotonic_clock::time_point(now.time_since_epoch());
  return stream;
}

void EventSchedulerScheduler::AddEventScheduler(EventScheduler *scheduler) {
  CHECK(std::find(schedulers_.begin(), schedulers_.end(), scheduler) ==
        schedulers_.end());
  CHECK(scheduler->scheduler_scheduler_ == nullptr);

  schedulers_.emplace_back(scheduler);
  scheduler->scheduler_scheduler_ = this;
}

void EventSchedulerScheduler::RunFor(distributed_clock::duration duration) {
  distributed_clock::time_point end_time = now_ + duration;
  logging::ScopedLogRestorer prev_logger;
  RunOnRun();

  // Run all the sub-event-schedulers.
  while (is_running_) {
    std::tuple<distributed_clock::time_point, EventScheduler *> oldest_event =
        OldestEvent();
    // No events left, bail.
    if (std::get<0>(oldest_event) == distributed_clock::max_time ||
        std::get<0>(oldest_event) > end_time) {
      is_running_ = false;
      break;
    }

    // We get to pick our tradeoffs here.  Either we assume that there are no
    // backward step changes in our time function for each node, or we have to
    // let time go backwards.  This backwards time jump should be small, so we
    // can check for it and bound it.
    CHECK_LE(now_, std::get<0>(oldest_event) + std::chrono::milliseconds(100))
        << ": Simulated time went backwards by too much.  Please investigate.";
    now_ = std::get<0>(oldest_event);

    std::get<1>(oldest_event)->CallOldestEvent();
  }

  now_ = end_time;
}

void EventSchedulerScheduler::Run() {
  logging::ScopedLogRestorer prev_logger;
  RunOnRun();
  // Run all the sub-event-schedulers.
  while (is_running_) {
    std::tuple<distributed_clock::time_point, EventScheduler *> oldest_event =
        OldestEvent();
    // No events left, bail.
    if (std::get<0>(oldest_event) == distributed_clock::max_time) {
      break;
    }

    // We get to pick our tradeoffs here.  Either we assume that there are no
    // backward step changes in our time function for each node, or we have to
    // let time go backwards.  This backwards time jump should be small, so we
    // can check for it and bound it.
    CHECK_LE(now_, std::get<0>(oldest_event) + std::chrono::milliseconds(100))
        << ": Simulated time went backwards by too much.  Please investigate.";
    now_ = std::get<0>(oldest_event);

    std::get<1>(oldest_event)->CallOldestEvent();
  }

  is_running_ = false;
}

std::tuple<distributed_clock::time_point, EventScheduler *>
EventSchedulerScheduler::OldestEvent() {
  distributed_clock::time_point min_event_time = distributed_clock::max_time;
  EventScheduler *min_scheduler = nullptr;

  // TODO(austin): Don't linearly search...  But for N=3, it is probably the
  // fastest way to do this.
  for (EventScheduler *scheduler : schedulers_) {
    const monotonic_clock::time_point monotonic_event_time =
        scheduler->OldestEvent();
    if (monotonic_event_time != monotonic_clock::max_time) {
      const distributed_clock::time_point event_time =
          scheduler->ToDistributedClock(monotonic_event_time);
      if (event_time < min_event_time) {
        min_event_time = event_time;
        min_scheduler = scheduler;
      }
    }
  }

  return std::make_tuple(min_event_time, min_scheduler);
}

}  // namespace aos
