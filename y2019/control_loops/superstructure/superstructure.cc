#include "y2019/control_loops/superstructure/superstructure.h"

#include "aos/controls/control_loops.q.h"
#include "frc971/control_loops/control_loops.q.h"

namespace y2019 {
namespace control_loops {
namespace superstructure {

Superstructure::Superstructure(::aos::EventLoop *event_loop,
                               const ::std::string &name)
    : aos::controls::ControlLoop<SuperstructureQueue>(event_loop, name) {}

void Superstructure::RunIteration(
    const SuperstructureQueue::Goal *unsafe_goal,
    const SuperstructureQueue::Position *position,
    SuperstructureQueue::Output *output,
    SuperstructureQueue::Status *status) {
  (void)unsafe_goal;
  (void)position;
  (void)output;
  (void)status;

  if (WasReset()) {
    LOG(ERROR, "WPILib reset, restarting\n");
  }
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2019
