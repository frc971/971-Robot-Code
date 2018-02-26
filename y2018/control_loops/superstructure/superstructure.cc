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
                       &(position->left_intake),
                       output != nullptr ? &(output->left_intake) : nullptr,
                       &(status->left_intake));

  intake_right_.Iterate(unsafe_goal != nullptr
                            ? &(unsafe_goal->intake.right_intake_angle)
                            : nullptr,
                        &(position->right_intake),
                        output != nullptr ? &(output->right_intake) : nullptr,
                        &(status->right_intake));

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
    const double roller_voltage = ::std::max(
        -kMaxIntakeRollerVoltage, ::std::min(unsafe_goal->intake.roller_voltage,
                                             kMaxIntakeRollerVoltage));
    constexpr int kReverseTime = 15;
    if (unsafe_goal->intake.roller_voltage < 0.0) {
      output->left_intake.voltage_rollers = roller_voltage;
      output->right_intake.voltage_rollers = roller_voltage;
      rotation_state_ = RotationState::NOT_ROTATING;
      rotation_count_ = 0;
    } else {
      switch (rotation_state_) {
        case RotationState::NOT_ROTATING:
          if (position->left_intake.beam_break) {
            rotation_state_ = RotationState::ROTATING_RIGHT;
            rotation_count_ = kReverseTime;
            break;
          } else if (position->right_intake.beam_break) {
            rotation_state_ = RotationState::ROTATING_LEFT;
            rotation_count_ = kReverseTime;
            break;
          } else {
            break;
          }
        case RotationState::ROTATING_LEFT:
          if (position->right_intake.beam_break) {
            rotation_count_ = kReverseTime;
          } else {
            --rotation_count_;
          }
          if (rotation_count_ == 0) {
            rotation_state_ = RotationState::NOT_ROTATING;
          }
          break;
        case RotationState::ROTATING_RIGHT:
          if (position->left_intake.beam_break) {
            rotation_count_ = kReverseTime;
          } else {
            --rotation_count_;
          }
          if (rotation_count_ == 0) {
            rotation_state_ = RotationState::NOT_ROTATING;
          }
          break;
      }

      switch (rotation_state_) {
        case RotationState::NOT_ROTATING:
          output->left_intake.voltage_rollers = roller_voltage;
          output->right_intake.voltage_rollers = roller_voltage;
          break;
        case RotationState::ROTATING_LEFT:
          output->left_intake.voltage_rollers = roller_voltage;
          output->right_intake.voltage_rollers = -roller_voltage;
          break;
        case RotationState::ROTATING_RIGHT:
          output->left_intake.voltage_rollers = -roller_voltage;
          output->right_intake.voltage_rollers = roller_voltage;
          break;
      }
    }
  }
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
