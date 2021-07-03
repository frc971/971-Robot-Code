#include "frc971/codelab/basic.h"

namespace frc971 {
namespace codelab {

Basic::Basic(::aos::EventLoop *event_loop, const ::std::string &name)
    : frc971::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                    name) {}

void Basic::RunIteration(const Goal *goal, const Position *position,
                         aos::Sender<Output>::Builder *output,
                         aos::Sender<Status>::Builder *status) {

  // FIX HERE: Set the intake_voltage to 12 Volts when
  // intake is requested (via intake in goal). Make sure not to set
  // the motor to anything but 0 V when the limit_sensor is pressed.

  // This line tells the compiler to to ignore the fact that goal and
  // position are not used in the code. You will need to read these messages
  // and use their values to determine the necessary output and status.
  (void)goal, (void)position;

  if (output) {
    Output::Builder builder = output->MakeBuilder<Output>();

    // FIX HERE: As of now, this sets the intake voltage to 0 in
    // all circumstances. Add to this code to output a different
    // intake voltage depending on the circumstances to make the
    // tests pass.
    builder.add_intake_voltage(0.0);

    // Ignore the return value of Send
    (void)output->Send(builder.Finish());
  }

  if (status) {
    Status::Builder builder = status->MakeBuilder<Status>();
    // FIX HERE: Fill out the Status message! In order to fill the
    // information in the message, use the add_<name of the field>() method
    // on the builder, just like we do with the Output message above.
    // Look at the definition of Status in basic_status.fbs to find
    // the name of the field.

    // Ignore the return value of Send
    (void)status->Send(builder.Finish());
  }
}

}  // namespace codelab
}  // namespace frc971
