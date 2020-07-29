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
  (void)goal, (void)position;

  if (output) {
    Output::Builder builder = output->MakeBuilder<Output>();
    // TODO(you): Fill out the voltage with a real voltage based on the
    // Goal/Position messages.
    builder.add_intake_voltage(0.0);

    output->Send(builder.Finish());
  }

  if (status) {
    Status::Builder builder = status->MakeBuilder<Status>();
    // TODO(you): Fill out the Status message! In order to populate fields, use
    // the add_field_name() method on the builder, just like we do with the
    // Output message above.

    status->Send(builder.Finish());
  }
}

}  // namespace codelab
}  // namespace frc971
