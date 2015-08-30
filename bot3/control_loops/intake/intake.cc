#include "bot3/control_loops/intake/intake.h"

#include "bot3/control_loops/intake/intake.q.h"

namespace bot3 {
namespace control_loops {

Intake::Intake(control_loops::IntakeQueue *intake)
    : aos::controls::ControlLoop<control_loops::IntakeQueue>(intake) {}

void Intake::RunIteration(
    const control_loops::IntakeQueue::Goal *goal,
    const control_loops::IntakeQueue::Position * /*position*/,
    control_loops::IntakeQueue::Output *output,
    control_loops::IntakeQueue::Status * /*status*/) {

  if (output != nullptr) {
    output->Zero();

    const int16_t intake_movement = goal->movement;

    if (intake_movement > 0) {
      // Suck.
      output->intake = kIntakeVoltageFullPower;
    } else if (intake_movement < 0) {
      // Spit.
      output->intake = -kIntakeVoltageFullPower;
    } else {
      // Stationary.
      output->intake = 0.0;
    }
  }
}

}  // namespace control_loops
}  // namespace bot3
