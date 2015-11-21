#include "y2015_bot3/control_loops/intake/intake.h"

#include "y2015_bot3/control_loops/intake/intake.q.h"

namespace y2015_bot3 {
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

    if (goal != nullptr) {
      output->intake = goal->movement;
      output->claw_closed = goal->claw_closed;
    } else {
      output->intake = 0.0;
      output->claw_closed = false;
    }
  }
}

}  // namespace control_loops
}  // namespace y2015_bot3
