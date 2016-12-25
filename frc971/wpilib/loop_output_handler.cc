#include "frc971/wpilib/loop_output_handler.h"

#include <sys/timerfd.h>

#include <chrono>
#include <functional>
#include <thread>

#include "aos/linux_code/init.h"
#include "aos/common/messages/robot_state.q.h"

namespace frc971 {
namespace wpilib {

namespace chrono = ::std::chrono;

LoopOutputHandler::LoopOutputHandler(::std::chrono::nanoseconds timeout)
    : watchdog_(this, timeout) {}

void LoopOutputHandler::operator()() {
  ::std::thread watchdog_thread(::std::ref(watchdog_));
  ::aos::SetCurrentThreadName("OutputHandler");

  ::aos::SetCurrentThreadRealtimePriority(30);
  while (run_) {
    no_joystick_state_.Print();
    fake_joystick_state_.Print();
    Read();
    ::aos::joystick_state.FetchLatest();
    if (!::aos::joystick_state.get()) {
      LOG_INTERVAL(no_joystick_state_);
      continue;
    }
    if (::aos::joystick_state->fake) {
      LOG_INTERVAL(fake_joystick_state_);
      continue;
    }

    watchdog_.Reset();
    Write();
  }

  Stop();

  watchdog_.Quit();
  watchdog_thread.join();
}

LoopOutputHandler::Watchdog::Watchdog(LoopOutputHandler *handler,
                                      ::std::chrono::nanoseconds timeout)
    : handler_(handler),
      timeout_(timeout),
      timerfd_(timerfd_create(CLOCK_MONOTONIC, 0)) {
  if (timerfd_.get() == -1) {
    PLOG(FATAL, "timerfd_create(CLOCK_MONOTONIC, 0)");
  }
}

void LoopOutputHandler::Watchdog::operator()() {
  ::aos::SetCurrentThreadRealtimePriority(35);
  ::aos::SetCurrentThreadName("OutputWatchdog");
  uint8_t buf[8];
  while (run_) {
    PCHECK(read(timerfd_.get(), buf, sizeof(buf)));
    handler_->Stop();
  }
}

void LoopOutputHandler::Watchdog::Reset() {
  itimerspec value = itimerspec();
  value.it_value.tv_sec = chrono::duration_cast<chrono::seconds>(timeout_).count();
  value.it_value.tv_nsec =
      chrono::duration_cast<chrono::nanoseconds>(
          timeout_ - chrono::seconds(value.it_value.tv_sec))
          .count();
  PCHECK(timerfd_settime(timerfd_.get(), 0, &value, nullptr));
}

}  // namespace wpilib
}  // namespace frc971
