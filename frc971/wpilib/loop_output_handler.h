#ifndef FRC971_WPILIB_LOOP_OUTPUT_HANDLER_H_
#define FRC971_WPILIB_LOOP_OUTPUT_HANDLER_H_

#include <atomic>

#include "aos/common/scoped_fd.h"
#include "aos/common/time.h"
#include "aos/common/util/log_interval.h"

namespace frc971 {
namespace wpilib {

// Handles sending the output from a single control loop to the hardware.
//
// This class implements stopping motors when no new values are received for too
// long efficiently.
//
// The intended use is to have a subclass for each loop which implements the
// pure virtual methods and is then run in a separate thread. The operator()
// loops writing values until Quit() is called.
class LoopOutputHandler {
 public:
  LoopOutputHandler(
      const ::aos::time::Time &timeout = ::aos::time::Time::InSeconds(0.10));

  void Quit() { run_ = false; }

  void operator()();

 protected:
  // Read a message from the appropriate queue.
  // This must block.
  virtual void Read() = 0;

  // Send the output from the appropriate queue to the hardware.
  // Read() will always be called at least once before per invocation of this.
  virtual void Write() = 0;

  // Stop all outputs. This will be called in a separate thread (if at all).
  // The subclass implementation should handle any appropriate logging.
  // This will be called exactly once if Read() blocks for too long and once
  // after Quit is called.
  virtual void Stop() = 0;

 private:
  // The thread that actually contains a timerfd to implement timeouts. The
  // idea is to have a timerfd that is repeatedly set to the timeout expiration
  // in the future so it will never actually expire unless it is not reset for
  // too long.
  //
  // This class nicely encapsulates the raw fd and manipulating it. Its
  // operator() loops until Quit() is called, calling Stop() on its associated
  // LoopOutputHandler whenever the timerfd expires.
  class Watchdog {
   public:
    Watchdog(LoopOutputHandler *handler, const ::aos::time::Time &timeout);

    void operator()();

    void Reset();

    void Quit() { run_ = false; }

   private:
    LoopOutputHandler *const handler_;

    const ::aos::time::Time timeout_;

    ::aos::ScopedFD timerfd_;

    ::std::atomic<bool> run_{true};
  };

  Watchdog watchdog_;

  ::std::atomic<bool> run_{true};

  ::aos::util::SimpleLogInterval no_joystick_state_ =
      ::aos::util::SimpleLogInterval(::aos::time::Time::InSeconds(0.5), INFO,
                                     "no joystick state -> not outputting");
  ::aos::util::SimpleLogInterval fake_joystick_state_ =
      ::aos::util::SimpleLogInterval(::aos::time::Time::InSeconds(0.5), DEBUG,
                                     "fake joystick state -> not outputting");
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_LOOP_OUTPUT_HANDLER_H_
