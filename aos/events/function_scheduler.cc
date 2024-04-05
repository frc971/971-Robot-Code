#include "aos/events/function_scheduler.h"

namespace aos {

FunctionScheduler::FunctionScheduler(aos::EventLoop *event_loop)
    : event_loop_(event_loop), timer_(event_loop_->AddTimer([this]() {
        RunFunctions(event_loop_->context().monotonic_event_time);
      })) {
  timer_->set_name("function_timer");
  event_loop_->OnRun(
      [this]() { RunFunctions(event_loop_->context().monotonic_event_time); });
}

void FunctionScheduler::ScheduleAt(std::function<void()> &&function,
                                   aos::monotonic_clock::time_point time) {
  functions_.insert(std::make_pair(time, std::move(function)));
  timer_->Schedule(functions_.begin()->first);
}

void FunctionScheduler::RunFunctions(aos::monotonic_clock::time_point now) {
  while (true) {
    if (functions_.empty()) return;
    if (functions_.begin()->first > now) {
      break;
    }
    CHECK_EQ(functions_.begin()->first, now);

    functions_.begin()->second();
    functions_.erase(functions_.begin());
  }
  timer_->Schedule(functions_.begin()->first);
}

}  // namespace aos
