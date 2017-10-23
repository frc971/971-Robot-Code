#include "y2017_bot3/control_loops/superstructure/superstructure.h"

#include "aos/common/commonmath.h"
#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"

namespace y2017_bot3 {
namespace control_loops {
namespace superstructure {

Superstructure::Superstructure(
    control_loops::SuperstructureQueue *superstructure_queue)
    : aos::controls::ControlLoop<control_loops::SuperstructureQueue>(
          superstructure_queue) {}

void Superstructure::RunIteration(
    const control_loops::SuperstructureQueue::Goal *unsafe_goal,
    const control_loops::SuperstructureQueue::Position * /*position*/,
    control_loops::SuperstructureQueue::Output *output,
    control_loops::SuperstructureQueue::Status * /*status*/) {
  // Write out all the voltages.
  if (output) {
    output->voltage_rollers = 0.0;
    output->hanger_voltage = 0.0;
    output->fingers_out = false;
    if (unsafe_goal) {
      // Intake.
      output->voltage_rollers = unsafe_goal->voltage_rollers;

      output->fingers_out = unsafe_goal->fingers_out;

      output->hanger_voltage = unsafe_goal->hanger_voltage;
    }
  }
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017_bot3
