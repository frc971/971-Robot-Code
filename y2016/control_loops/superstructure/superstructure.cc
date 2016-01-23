#include "y2016/control_loops/superstructure/superstructure.h"

#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"

namespace y2016 {
namespace control_loops {

Superstructure::Superstructure(
    control_loops::SuperstructureQueue *my_superstructure)
    : aos::controls::ControlLoop<control_loops::SuperstructureQueue>(
          my_superstructure) {}

void Superstructure::RunIteration(
    const control_loops::SuperstructureQueue::Goal *goal,
    const control_loops::SuperstructureQueue::Position *position,
    ::aos::control_loops::Output *output,
    control_loops::SuperstructureQueue::Status *status) {
  (void)goal;
  (void)position;
  (void)output;
  (void)status;
}

}  // namespace control_loops
}  // namespace y2016
