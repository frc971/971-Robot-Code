#include "y2018/control_loops/superstructure/superstructure.h"

#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"
#include "frc971/control_loops/control_loops.q.h"
#include "y2018/constants.h"
#include "y2018/control_loops/superstructure/intake/intake.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {

namespace {
// The maximum voltage the intake roller will be allowed to use.
constexpr double kMaxIntakeRollerVoltage = 12.0;
}  // namespace

Superstructure::Superstructure(
    control_loops::SuperstructureQueue *superstructure_queue)
    : aos::controls::ControlLoop<control_loops::SuperstructureQueue>(
          superstructure_queue),
      intake_left_(constants::GetValues().left_intake.zeroing),
      intake_right_(constants::GetValues().right_intake.zeroing) {}

void Superstructure::RunIteration(
    const control_loops::SuperstructureQueue::Goal *unsafe_goal,
    const control_loops::SuperstructureQueue::Position *position,
    control_loops::SuperstructureQueue::Output *output,
    control_loops::SuperstructureQueue::Status *status) {
  if (WasReset()) {
    LOG(ERROR, "WPILib reset, restarting\n");
    intake_left_.Reset();
    intake_right_.Reset();
    arm_.Reset();
  }

  intake_left_.Iterate(unsafe_goal != nullptr
                           ? &(unsafe_goal->intake.left_intake_angle)
                           : nullptr,
                       &(position->intake.left),
                       output != nullptr ? &(output->intake.left) : nullptr,
                       &(status->left_intake));

  intake_right_.Iterate(unsafe_goal != nullptr
                            ? &(unsafe_goal->intake.right_intake_angle)
                            : nullptr,
                        &(position->intake.right),
                        output != nullptr ? &(output->intake.right) : nullptr,
                        &(status->right_intake));

  intake_right_.Iterate(unsafe_goal != nullptr
                            ? &(unsafe_goal->intake.right_intake_angle)
                            : nullptr,
                        &(position->intake.right),
                        output != nullptr ? &(output->intake.right) : nullptr,
                        &(status->left_intake));

  arm_.Iterate(
      unsafe_goal != nullptr ? &(unsafe_goal->arm_goal_position) : nullptr,
      &(position->arm),
      output != nullptr ? &(output->voltage_proximal) : nullptr,
      output != nullptr ? &(output->voltage_distal) : nullptr,
      output != nullptr ? &(output->release_arm_brake) : nullptr,
      &(status->arm));

  status->estopped = status->left_intake.estopped ||
                     status->right_intake.estopped || status->arm.estopped;

  status->zeroed = status->left_intake.zeroed && status->right_intake.zeroed &&
                   status->arm.zeroed;

  if (output && unsafe_goal) {
    output->intake.left.voltage_rollers = ::std::max(
        -kMaxIntakeRollerVoltage, ::std::min(unsafe_goal->intake.roller_voltage,
                                             kMaxIntakeRollerVoltage));
    output->intake.right.voltage_rollers = ::std::max(
        -kMaxIntakeRollerVoltage, ::std::min(unsafe_goal->intake.roller_voltage,
                                             kMaxIntakeRollerVoltage));
  }
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
