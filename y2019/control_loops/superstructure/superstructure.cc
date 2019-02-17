#include "y2019/control_loops/superstructure/superstructure.h"

#include "aos/controls/control_loops.q.h"
#include "frc971/control_loops/control_loops.q.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"

namespace y2019 {
namespace control_loops {
namespace superstructure {

void Superstructure::HandleSuction(const SuctionGoal *unsafe_goal,
                                   float suction_pressure,
                                   SuperstructureQueue::Output *output,
                                   bool *has_piece) {
  constexpr double kPumpVoltage = 12.0;
  constexpr double kPumpHasPieceVoltage = 8.0;

  // TODO(austin): Low pass filter on pressure.
  *has_piece = suction_pressure < 0.70;

  if (unsafe_goal && output) {
    const bool evacuate = unsafe_goal->top || unsafe_goal->bottom;
    if (evacuate) {
      vacuum_count_ = 200;
    }
    // TODO(austin): High speed pump a bit longer after we detect we have the
    // game piece.
    // Once the vacuum evacuates, the pump speeds up because there is no
    // resistance.  So, we want to turn it down to save the pump from
    // overheating.
    output->pump_voltage =
        (vacuum_count_ > 0) ? (*has_piece ? kPumpHasPieceVoltage : kPumpVoltage)
                            : 0.0;
    output->intake_suction_top = unsafe_goal->top;
    output->intake_suction_bottom = unsafe_goal->bottom;
  }
  vacuum_count_ = ::std::max(0, vacuum_count_ - 1);
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

  intake_.Iterate(unsafe_goal != nullptr ? &(unsafe_goal->intake) : nullptr,
                  &(position->intake_joint),
                  output != nullptr ? &(output->intake_joint_voltage) : nullptr,
                  &(status->intake));

  stilts_.Iterate(unsafe_goal != nullptr ? &(unsafe_goal->stilts) : nullptr,
                  &(position->stilts),
                  output != nullptr ? &(output->stilts_voltage) : nullptr,
                  &(status->stilts));

  HandleSuction(unsafe_goal != nullptr ? &(unsafe_goal->suction) : nullptr,
                position->suction_pressure, output, &(status->has_piece));

  status->zeroed = status->elevator.zeroed && status->wrist.zeroed &&
                   status->intake.zeroed && status->stilts.zeroed;

  status->estopped = status->elevator.estopped || status->wrist.estopped ||
                     status->intake.estopped || status->stilts.estopped;

  if (output) {
    if (unsafe_goal && status->intake.position > kMinIntakeAngleForRollers) {
      output->intake_roller_voltage = unsafe_goal->roller_voltage;
    } else {
      output->intake_roller_voltage = 0.0;
    }
  }

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
