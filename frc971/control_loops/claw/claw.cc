#include "frc971/control_loops/claw/claw.h"

#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "frc971/control_loops/claw/claw_motor_plant.h"

namespace frc971 {
namespace control_loops {

Claw::Claw(control_loops::ClawQueue *claw)
    : aos::controls::ControlLoop<control_loops::ClawQueue>(claw),
      claw_loop_(new StateFeedbackLoop<2, 1, 1>(MakeClawLoop())) {}

void Claw::RunIteration(
    const control_loops::ClawQueue::Goal * /*goal*/,
    const control_loops::ClawQueue::Position * /*position*/,
    control_loops::ClawQueue::Output * /*output*/,
    control_loops::ClawQueue::Status * /*status*/) {

  LOG(DEBUG, "Hi Brian!");
}

}  // namespace control_loops
}  // namespace frc971
