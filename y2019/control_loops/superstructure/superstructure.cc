#include "y2019/control_loops/superstructure/superstructure.h"

#include "aos/controls/control_loops.q.h"
#include "frc971/control_loops/control_loops.q.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"

namespace y2019 {
namespace control_loops {
namespace superstructure {

void suction_cups(
    const SuperstructureQueue::Goal *unsafe_goal,
    SuperstructureQueue::Output *output) {
  const double on_voltage = 12.0;

  if(unsafe_goal && output) {
    if(unsafe_goal->suction.top || unsafe_goal->suction.bottom) {
      output->pump_voltage = on_voltage;
    }
  }
}

Superstructure::Superstructure(::aos::EventLoop *event_loop,
                               const ::std::string &name)
    : aos::controls::ControlLoop<SuperstructureQueue>(event_loop, name),
      elevator_(constants::GetValues().elevator.subsystem_params),
      wrist_(constants::GetValues().wrist.subsystem_params),
      intake_(constants::GetValues().intake),
      stilts_(constants::GetValues().stilts.subsystem_params) {}

void Superstructure::RunIteration(const SuperstructureQueue::Goal *unsafe_goal,
                                  const SuperstructureQueue::Position *position,
                                  SuperstructureQueue::Output *output,
                                  SuperstructureQueue::Status *status) {
  if (WasReset()) {
    LOG(ERROR, "WPILib reset, restarting\n");
    elevator_.Reset();
    wrist_.Reset();
    intake_.Reset();
    stilts_.Reset();
  }

  elevator_.Iterate(unsafe_goal != nullptr ? &(unsafe_goal->elevator) : nullptr,
                    &(position->elevator),
                    output != nullptr ? &(output->elevator_voltage) : nullptr,
                    &(status->elevator));

  wrist_.Iterate(unsafe_goal != nullptr ? &(unsafe_goal->wrist) : nullptr,
                 &(position->wrist),
                 output != nullptr ? &(output->wrist_voltage) : nullptr,
                 &(status->wrist));

  intake_.Iterate(
      unsafe_goal != nullptr ? &(unsafe_goal->intake) : nullptr,
      &(position->intake_joint),
      output != nullptr ? &(output->intake_joint_voltage) : nullptr,
      &(status->intake));

  stilts_.Iterate(unsafe_goal != nullptr ? &(unsafe_goal->stilts) : nullptr,
                  &(position->stilts),
                  output != nullptr ? &(output->stilts_voltage) : nullptr,
                  &(status->stilts));

  status->zeroed = status->elevator.zeroed && status->wrist.zeroed &&
                   status->intake.zeroed && status->stilts.zeroed;

  status->estopped = status->elevator.estopped || status->wrist.estopped ||
                     status->intake.estopped || status->stilts.estopped;

  // TODO(theo) move these up when Iterate() is split
  // update the goals
  collision_avoidance_.UpdateGoal(status, unsafe_goal);

  elevator_.set_min_position(collision_avoidance_.min_elevator_goal());
  wrist_.set_min_position(collision_avoidance_.min_wrist_goal());
  wrist_.set_max_position(collision_avoidance_.max_wrist_goal());
  intake_.set_min_position(collision_avoidance_.min_intake_goal());
  intake_.set_max_position(collision_avoidance_.max_intake_goal());
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2019
