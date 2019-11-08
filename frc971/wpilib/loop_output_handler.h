#ifndef FRC971_WPILIB_LOOP_OUTPUT_HANDLER_H_
#define FRC971_WPILIB_LOOP_OUTPUT_HANDLER_H_

#include <atomic>
#include <chrono>

#include "aos/events/event_loop.h"
#include "aos/scoped/scoped_fd.h"
#include "aos/time/time.h"
#include "aos/util/log_interval.h"

namespace frc971 {
namespace wpilib {

// Handles sending the output from a single control loop to the hardware.
//
// This class implements stopping motors when no new values are received for too
// long efficiently.
//
// The intended use is to have a subclass for each loop which implements the
// pure virtual methods.
template <typename T>
class LoopOutputHandler {
 public:
  LoopOutputHandler(
      ::aos::EventLoop *event_loop, const ::std::string &name,
      ::std::chrono::nanoseconds timeout = ::std::chrono::milliseconds(100))
      : event_loop_(event_loop), timeout_(timeout) {
    event_loop_->SetRuntimeRealtimePriority(30);
    // TODO(austin): Name thread.
    event_loop_->MakeWatcher(name, [this](const T &t) {
      // Push the watchdog out a bit further.
      timer_handler_->Setup(event_loop_->monotonic_now() + timeout_);
      Write(t);
    });

    // TODO(austin): Set name.
    timer_handler_ = event_loop_->AddTimer([this]() { Stop(); });

    event_loop_->OnRun([this]() {
      timer_handler_->Setup(event_loop_->monotonic_now() + timeout_);
    });
  }

  // Note, all subclasses should call Stop.
  virtual ~LoopOutputHandler() {}

 protected:
  // Send the output from the appropriate queue to the hardware.
  virtual void Write(const T &t) = 0;

  // Stop all outputs. This will be called in a separate thread (if at all).
  // The subclass implementation should handle any appropriate logging.
  // This will be called exactly once if Read() blocks for too long and once
  // after Quit is called.
  virtual void Stop() = 0;

  // Returns a pointer to the event loop.
  ::aos::EventLoop *event_loop() { return event_loop_; }

 private:
  ::aos::EventLoop *event_loop_;
  const ::std::chrono::nanoseconds timeout_;
  ::aos::TimerHandler *timer_handler_;
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_LOOP_OUTPUT_HANDLER_H_
