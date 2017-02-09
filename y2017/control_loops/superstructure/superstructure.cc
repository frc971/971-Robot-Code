#include "y2017/control_loops/superstructure/superstructure.h"

#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"
#include "y2017/constants.h"
#include "y2017/control_loops/superstructure/hood/hood.h"
#include "y2017/control_loops/superstructure/turret/turret.h"
#include "y2017/control_loops/superstructure/intake/intake.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {

Superstructure::Superstructure(
    control_loops::SuperstructureQueue *superstructure_queue)
    : aos::controls::ControlLoop<control_loops::SuperstructureQueue>(
          superstructure_queue) {}

void Superstructure::RunIteration(
    const control_loops::SuperstructureQueue::Goal *unsafe_goal,
    const control_loops::SuperstructureQueue::Position *position,
    control_loops::SuperstructureQueue::Output *output,
    control_loops::SuperstructureQueue::Status *status) {
  if (WasReset()) {
    LOG(ERROR, "WPILib reset, restarting\n");
    hood_.Reset();
    turret_.Reset();
    intake_.Reset();
  }

  hood_.Iterate(unsafe_goal != nullptr ? &(unsafe_goal->hood) : nullptr,
                &(position->hood),
                output != nullptr ? &(output->voltage_hood) : nullptr,
                &(status->hood));

  turret_.Iterate(unsafe_goal != nullptr ? &(unsafe_goal->turret) : nullptr,
                &(position->turret),
                output != nullptr ? &(output->voltage_turret) : nullptr,
                &(status->turret));

  intake_.Iterate(unsafe_goal != nullptr ? &(unsafe_goal->intake) : nullptr,
                  &(position->intake),
                  output != nullptr ? &(output->voltage_intake) : nullptr,
                  &(status->intake));
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017
