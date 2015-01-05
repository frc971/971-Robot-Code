#include "frc971/wpilib/loop_output_handler.h"

#include <sys/timerfd.h>

#include <thread>
#include <functional>

#include "aos/linux_code/init.h"
#include "aos/common/messages/robot_state.q.h"

namespace frc971 {
namespace wpilib {

constexpr ::aos::time::Time LoopOutputHandler::kStopTimeout;

LoopOutputHandler::LoopOutputHandler() : watchdog_(this) {}

void LoopOutputHandler::operator()() {
  ::aos::SetCurrentThreadName("OutputHandler");
  ::std::thread watchdog_thread(::std::ref(watchdog_));

  ::aos::SetCurrentThreadRealtimePriority(30);
  while (run_) {
    no_robot_state_.Print();
    fake_robot_state_.Print();
    Read();
    ::aos::robot_state.FetchLatest();
    if (!::aos::robot_state.get()) {
      LOG_INTERVAL(no_robot_state_);
      continue;
    }
    if (::aos::robot_state->fake) {
      LOG_INTERVAL(fake_robot_state_);
      continue;
    }

    watchdog_.Reset();
    Write();
  }

  Stop();

  watchdog_.Quit();
  watchdog_thread.join();
}

LoopOutputHandler::Watchdog::Watchdog(LoopOutputHandler *handler)
    : handler_(handler),
      timerfd_(timerfd_create(::aos::time::Time::kDefaultClock, 0)) {
  if (timerfd_.get() == -1) {
    PLOG(FATAL, "timerfd_create(Time::kDefaultClock, 0)");
  }
  ::aos::SetCurrentThreadRealtimePriority(35);
  ::aos::SetCurrentThreadName("OutputWatchdog");
}

void LoopOutputHandler::Watchdog::operator()() {
  uint8_t buf[8];
  while (run_) {
    PCHECK(read(timerfd_.get(), buf, sizeof(buf)));
    handler_->Stop();
  }
}

void LoopOutputHandler::Watchdog::Reset() {
  itimerspec value = itimerspec();
  value.it_value = kStopTimeout.ToTimespec();
  PCHECK(timerfd_settime(timerfd_.get(), 0, &value, nullptr));
}

}  // namespace wpilib
}  // namespace frc971
