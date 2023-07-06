#include "aos/events/event_scheduler.h"

#include <algorithm>
#include <deque>

#include "aos/events/event_loop.h"
#include "aos/logging/implementations.h"

namespace aos {

EventScheduler::Token EventScheduler::Schedule(monotonic_clock::time_point time,
                                               Event *callback) {
  CHECK_LE(monotonic_clock::epoch(), time);
  return events_list_.emplace(time, callback);
}

void EventScheduler::Deschedule(EventScheduler::Token token) {
  // We basically want to DCHECK some nontrivial logic. Guard it with NDEBUG to
  // ensure the compiler realizes it's all unnecessary when not doing debug
  // checks.
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

std::pair<distributed_clock::time_point, monotonic_clock::time_point>
EventScheduler::OldestEvent() {
  // If we haven't started yet, schedule a special event for the epoch to allow
  // ourselves to boot.
  if (!called_started_) {
    if (!cached_epoch_) {
      cached_epoch_ = ToDistributedClock(monotonic_clock::epoch());
    }
    return std::make_pair(*cached_epoch_, monotonic_clock::epoch());
  }

  if (events_list_.empty()) {
    return std::make_pair(distributed_clock::max_time,
                          monotonic_clock::max_time);
  }

  const monotonic_clock::time_point monotonic_time =
      events_list_.begin()->first;
  if (cached_event_list_monotonic_time_ != monotonic_time) {
    cached_event_list_time_ = ToDistributedClock(monotonic_time);
    cached_event_list_monotonic_time_ = monotonic_time;
  }

  return std::make_pair(cached_event_list_time_, monotonic_time);
}

void EventScheduler::Shutdown() {
  CHECK(!is_running_);
  on_shutdown_();
}

void EventScheduler::Startup() {
  ++boot_count_;
  CHECK(!is_running_);
  MaybeRunOnStartup();
  CHECK(called_started_);
}

void EventScheduler::CallOldestEvent() {
  if (!called_started_) {
    // If we haven't started, start.
    MaybeRunOnStartup();
    MaybeRunOnRun();
    CHECK(called_started_);
    return;
  }
  CHECK(is_running_);
  CHECK_GT(events_list_.size(), 0u);
  auto iter = events_list_.begin();
  const logger::BootTimestamp t =
      FromDistributedClock(scheduler_scheduler_->distributed_now());
  VLOG(2) << "Got time back " << t;
  CHECK_EQ(t.boot, boot_count_);
  CHECK_EQ(t.time, iter->first) << ": Time is wrong on node " << node_index_;

  Event *callback = iter->second;
  events_list_.erase(iter);
  callback->Handle();

  converter_->ObserveTimePassed(scheduler_scheduler_->distributed_now());
}

void EventScheduler::RunOnRun() {
  CHECK(is_running_);
  while (!on_run_.empty()) {
    std::function<void()> fn = std::move(*on_run_.begin());
    on_run_.erase(on_run_.begin());
    fn();
  }
}

void EventScheduler::RunOnStartup() noexcept {
  while (!on_startup_.empty()) {
    CHECK(!is_running_);
    std::function<void()> fn = std::move(*on_startup_.begin());
    on_startup_.erase(on_startup_.begin());
    fn();
  }
}

void EventScheduler::RunStarted() {
  CHECK(!is_running_);
  if (started_) {
    started_();
  }
  is_running_ = true;
}

void EventScheduler::MaybeRunStopped() {
  CHECK(is_running_);
  is_running_ = false;
  if (called_started_) {
    called_started_ = false;
    if (stopped_) {
      stopped_();
    }
  }
}

void EventScheduler::MaybeRunOnStartup() {
  CHECK(!called_started_);
  CHECK(!is_running_);
  const logger::BootTimestamp t =
      FromDistributedClock(scheduler_scheduler_->distributed_now());
  if (t.boot == boot_count_ && t.time >= monotonic_clock::epoch()) {
    called_started_ = true;
    RunOnStartup();
  }
}

void EventScheduler::MaybeRunOnRun() {
  if (called_started_) {
    RunStarted();
    RunOnRun();
  }
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
  CHECK_EQ(scheduler->node_index(), schedulers_.size());

  schedulers_.emplace_back(scheduler);
  scheduler->scheduler_scheduler_ = this;
}

void EventSchedulerScheduler::MaybeRunStopped() {
  CHECK(!is_running_);
  for (EventScheduler *scheduler : schedulers_) {
    if (scheduler->is_running()) {
      scheduler->MaybeRunStopped();
    }
  }
}

bool EventSchedulerScheduler::RunUntil(
    realtime_clock::time_point end_time, EventScheduler *scheduler,
    std::function<std::chrono::nanoseconds()> fn_realtime_offset) {
  logging::ScopedLogRestorer prev_logger;
  MaybeRunOnStartup();

  bool reached_end_time = false;

  RunMaybeRealtimeLoop([this, scheduler, end_time, fn_realtime_offset,
                        &reached_end_time]() {
    std::tuple<distributed_clock::time_point, EventScheduler *> oldest_event =
        OldestEvent();
    aos::distributed_clock::time_point oldest_event_time_distributed =
        std::get<0>(oldest_event);
    logger::BootTimestamp test_time_monotonic =
        scheduler->FromDistributedClock(oldest_event_time_distributed);
    realtime_clock::time_point oldest_event_realtime(
        test_time_monotonic.time_since_epoch() + fn_realtime_offset());

    if ((std::get<0>(oldest_event) == distributed_clock::max_time) ||
        (oldest_event_realtime > end_time &&
         (reboots_.empty() ||
          std::get<0>(reboots_.front()) > oldest_event_time_distributed))) {
      is_running_ = false;
      reached_end_time = true;

      // We have to nudge our time back to the distributed time
      // corresponding to our desired realtime time.
      const monotonic_clock::time_point end_monotonic =
          monotonic_clock::epoch() + end_time.time_since_epoch() -
          fn_realtime_offset();
      const aos::distributed_clock::time_point end_time_distributed =
          scheduler->ToDistributedClock(end_monotonic);

      now_ = end_time_distributed;

      return;
    }

    if (!reboots_.empty() &&
        std::get<0>(reboots_.front()) <= std::get<0>(oldest_event)) {
      // Reboot is next.
      CHECK_LE(now_,
               std::get<0>(reboots_.front()) + std::chrono::nanoseconds(1))
          << ": Simulated time went backwards by too much.  Please "
             "investigate.";
      now_ = std::get<0>(reboots_.front());
      Reboot();
      reboots_.erase(reboots_.begin());
      return;
    }

    // We get to pick our tradeoffs here.  Either we assume that there are
    // no backward step changes in our time function for each node, or we
    // have to let time go backwards.  We currently only really see this
    // happen when 2 events are scheduled for "now", time changes, and
    // there is a nanosecond or two of rounding due to integer math.
    //
    // //aos/events/logging:logger_test triggers this.
    CHECK_LE(now_, std::get<0>(oldest_event) + std::chrono::nanoseconds(1))
        << ": Simulated time went backwards by too much.  Please "
           "investigate.";

    now_ = std::get<0>(oldest_event);

    std::get<1>(oldest_event)->CallOldestEvent();
  });

  MaybeRunStopped();

  return reached_end_time;
}

void EventSchedulerScheduler::Reboot() {
  const std::vector<logger::BootTimestamp> &times =
      std::get<1>(reboots_.front());
  CHECK_EQ(times.size(), schedulers_.size());

  VLOG(1) << "Rebooting at " << now_;
  for (const auto &time : times) {
    VLOG(1) << "  " << time;
  }

  is_running_ = false;

  // Shut everything down.
  std::vector<size_t> rebooted;
  for (size_t node_index = 0; node_index < schedulers_.size(); ++node_index) {
    if (schedulers_[node_index]->boot_count() == times[node_index].boot) {
      continue;
    } else {
      rebooted.emplace_back(node_index);
      CHECK_EQ(schedulers_[node_index]->boot_count() + 1,
               times[node_index].boot);
      schedulers_[node_index]->MaybeRunStopped();
      schedulers_[node_index]->Shutdown();
    }
  }

  // And start it back up again to reboot.  When something starts back up
  // (especially message_bridge), it could try to send stuff out.  We want
  // to move everything over to the new boot before doing that.
  for (const size_t node_index : rebooted) {
    schedulers_[node_index]->Startup();
  }
  for (const size_t node_index : rebooted) {
    schedulers_[node_index]->MaybeRunOnRun();
  }
  is_running_ = true;
}

void EventSchedulerScheduler::RunFor(distributed_clock::duration duration) {
  distributed_clock::time_point end_time = now_ + duration;
  logging::ScopedLogRestorer prev_logger;
  MaybeRunOnStartup();

  // Run all the sub-event-schedulers.
  RunMaybeRealtimeLoop([this, end_time]() {
    std::tuple<distributed_clock::time_point, EventScheduler *> oldest_event =
        OldestEvent();
    if (!reboots_.empty() &&
        std::get<0>(reboots_.front()) <= std::get<0>(oldest_event)) {
      // Reboot is next.
      if (std::get<0>(reboots_.front()) > end_time) {
        // Reboot is after our end time, give up.
        is_running_ = false;
        return;
      }

      CHECK_LE(now_,
               std::get<0>(reboots_.front()) + std::chrono::nanoseconds(1))
          << ": Simulated time went backwards by too much.  Please "
             "investigate.";
      now_ = std::get<0>(reboots_.front());
      Reboot();
      reboots_.erase(reboots_.begin());
      return;
    }

    // No events left, bail.
    if (std::get<0>(oldest_event) == distributed_clock::max_time ||
        std::get<0>(oldest_event) > end_time) {
      is_running_ = false;
      return;
    }

    // We get to pick our tradeoffs here.  Either we assume that there are no
    // backward step changes in our time function for each node, or we have to
    // let time go backwards.  We currently only really see this happen when 2
    // events are scheduled for "now", time changes, and there is a nanosecond
    // or two of rounding due to integer math.
    //
    // //aos/events/logging:logger_test triggers this.
    CHECK_LE(now_, std::get<0>(oldest_event) + std::chrono::nanoseconds(1))
        << ": Simulated time went backwards by too much.  Please investigate.";
    // push time forwards
    now_ = std::get<0>(oldest_event);

    std::get<1>(oldest_event)->CallOldestEvent();
  });

  now_ = end_time;

  MaybeRunStopped();
}

void EventSchedulerScheduler::Run() {
  logging::ScopedLogRestorer prev_logger;
  MaybeRunOnStartup();

  // Run all the sub-event-schedulers.
  RunMaybeRealtimeLoop([this]() {
    std::tuple<distributed_clock::time_point, EventScheduler *> oldest_event =
        OldestEvent();
    if (!reboots_.empty() &&
        std::get<0>(reboots_.front()) <= std::get<0>(oldest_event)) {
      // Reboot is next.
      CHECK_LE(now_,
               std::get<0>(reboots_.front()) + std::chrono::nanoseconds(1))
          << ": Simulated time went backwards by too much.  Please "
             "investigate.";
      now_ = std::get<0>(reboots_.front());
      Reboot();
      reboots_.erase(reboots_.begin());
      return;
    }
    // No events left, bail.
    if (std::get<0>(oldest_event) == distributed_clock::max_time) {
      is_running_ = false;
      return;
    }

    // We get to pick our tradeoffs here.  Either we assume that there are no
    // backward step changes in our time function for each node, or we have to
    // let time go backwards.  We currently only really see this happen when 2
    // events are scheduled for "now", time changes, and there is a nanosecond
    // or two of rounding due to integer math.
    //
    // //aos/events/logging:logger_test triggers this.
    CHECK_LE(now_, std::get<0>(oldest_event) + std::chrono::nanoseconds(1))
        << ": Simulated time went backwards by too much.  Please investigate.";
    now_ = std::get<0>(oldest_event);

    std::get<1>(oldest_event)->CallOldestEvent();
  });

  MaybeRunStopped();
}

template <typename F>
void EventSchedulerScheduler::RunMaybeRealtimeLoop(F loop_body) {
  internal::TimerFd timerfd;
  CHECK_LT(0.0, replay_rate_) << "Replay rate must be positive.";
  distributed_clock::time_point last_distributed_clock =
      std::get<0>(OldestEvent());
  monotonic_clock::time_point last_monotonic_clock = monotonic_clock::now();
  timerfd.SetTime(last_monotonic_clock, std::chrono::seconds(0));
  epoll_.OnReadable(
      timerfd.fd(), [this, &last_distributed_clock, &last_monotonic_clock,
                     &timerfd, loop_body]() {
        const uint64_t read_result = timerfd.Read();
        if (!is_running_) {
          epoll_.Quit();
          return;
        }
        CHECK_EQ(read_result, 1u);
        // Call loop_body() at least once; if we are in infinite-speed replay,
        // we don't actually want/need the context switches from the epoll
        // setup, so just loop.
        // Note: The performance impacts of this code have not been carefully
        // inspected (e.g., how much does avoiding the context-switch help; does
        // the timerfd_settime call matter).
        // This is deliberately written to support the user changing replay
        // rates dynamically.
        do {
          loop_body();
          if (is_running_) {
            const monotonic_clock::time_point next_trigger =
                last_monotonic_clock +
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    (now_ - last_distributed_clock) / replay_rate_);
            timerfd.SetTime(next_trigger, std::chrono::seconds(0));
            last_monotonic_clock = next_trigger;
            last_distributed_clock = now_;
          } else {
            epoll_.Quit();
          }
        } while (replay_rate_ == std::numeric_limits<double>::infinity() &&
                 is_running_);
      });

  epoll_.Run();
  epoll_.DeleteFd(timerfd.fd());
}

std::tuple<distributed_clock::time_point, EventScheduler *>
EventSchedulerScheduler::OldestEvent() {
  distributed_clock::time_point min_event_time = distributed_clock::max_time;
  EventScheduler *min_scheduler = nullptr;

  // TODO(austin): Don't linearly search...  But for N=3, it is probably the
  // fastest way to do this.
  for (EventScheduler *scheduler : schedulers_) {
    const std::pair<distributed_clock::time_point, monotonic_clock::time_point>
        event_time = scheduler->OldestEvent();
    if (event_time.second != monotonic_clock::max_time) {
      if (event_time.first < min_event_time) {
        min_event_time = event_time.first;
        min_scheduler = scheduler;
      }
    }
  }

  if (min_scheduler) {
    VLOG(2) << "Oldest event " << min_event_time << " on scheduler "
            << min_scheduler->node_index_;
  }
  return std::make_tuple(min_event_time, min_scheduler);
}

void EventSchedulerScheduler::TemporarilyStopAndRun(std::function<void()> fn) {
  const bool was_running = is_running_;
  if (is_running_) {
    is_running_ = false;
    MaybeRunStopped();
  }
  fn();
  if (was_running) {
    MaybeRunOnStartup();
  }
}

void EventSchedulerScheduler::MaybeRunOnStartup() {
  is_running_ = true;
  for (EventScheduler *scheduler : schedulers_) {
    scheduler->MaybeRunOnStartup();
  }
  // We must trigger all the OnRun's *after* all the OnStartup callbacks are
  // triggered because that is the contract that we have stated.
  for (EventScheduler *scheduler : schedulers_) {
    scheduler->MaybeRunOnRun();
  }
}

}  // namespace aos
