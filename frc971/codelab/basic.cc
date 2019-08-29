#include "frc971/codelab/basic.h"

namespace frc971 {
namespace codelab {

Basic::Basic(::aos::EventLoop *event_loop, const ::std::string &name)
    : aos::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                 name) {}

void Basic::RunIteration(const Goal *goal, const Position *position,
                         aos::Sender<Output>::Builder *output,
                         aos::Sender<Status>::Builder *status) {
  // TODO(you): Set the intake_voltage to 12 Volts when
  // intake is requested (via intake in goal). Make sure not to set
  // the motor to anything but 0 V when the limit_sensor is pressed.

  // Ignore: These are to avoid clang warnings.
  (void)goal, (void)position, (void)output, (void)status;
}

}  // namespace codelab
}  // namespace frc971
