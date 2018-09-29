#include "y2018/control_loops/superstructure/superstructure.h"

#include <chrono>

#include "aos/controls/control_loops.q.h"
#include "aos/logging/logging.h"
#include "frc971/control_loops/control_loops.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2018/constants.h"
#include "y2018/control_loops/superstructure/intake/intake.h"
#include "y2018/status_light.q.h"
#include "y2018/vision/vision.q.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {

using ::aos::monotonic_clock;

namespace chrono = ::std::chrono;

namespace {
// The maximum voltage the intake roller will be allowed to use.
constexpr double kMaxIntakeRollerVoltage = 12.0;
}  // namespace

void SendColors(float red, float green, float blue) {
  auto new_status_light = status_light.MakeMessage();
  new_status_light->red = red;
  new_status_light->green = green;
  new_status_light->blue = blue;

  if (!new_status_light.Send()) {
    LOG(ERROR, "Failed to send lights.\n");
  }
}

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

  const double clipped_box_distance =
      ::std::min(1.0, ::std::max(0.0, position->box_distance));

  const double box_velocity =
      (clipped_box_distance - last_box_distance_) / 0.005;

  constexpr double kFilteredBoxVelocityAlpha = 0.02;
  filtered_box_velocity_ =
      box_velocity * kFilteredBoxVelocityAlpha +
      (1.0 - kFilteredBoxVelocityAlpha) * filtered_box_velocity_;
  status->filtered_box_velocity = filtered_box_velocity_;

  constexpr double kCenteringAngleGain = 0.0;
  const double left_intake_goal =
      ::std::min(
          arm_.max_intake_override(),
          (unsafe_goal == nullptr ? 0.0
                                  : unsafe_goal->intake.left_intake_angle)) +
      last_intake_center_error_ * kCenteringAngleGain;
  const double right_intake_goal =
      ::std::min(
          arm_.max_intake_override(),
          (unsafe_goal == nullptr ? 0.0
                                  : unsafe_goal->intake.right_intake_angle)) -
      last_intake_center_error_ * kCenteringAngleGain;

  intake_left_.Iterate(unsafe_goal != nullptr ? &(left_intake_goal) : nullptr,
                       &(position->left_intake),
                       output != nullptr ? &(output->left_intake) : nullptr,
                       &(status->left_intake));

  intake_right_.Iterate(unsafe_goal != nullptr ? &(right_intake_goal) : nullptr,
                        &(position->right_intake),
                        output != nullptr ? &(output->right_intake) : nullptr,
                        &(status->right_intake));

  const double intake_center_error =
      intake_right_.output_position() - intake_left_.output_position();
  last_intake_center_error_ = intake_center_error;

  const bool intake_clear_of_box =
      intake_left_.clear_of_box() && intake_right_.clear_of_box();

  bool open_claw = unsafe_goal != nullptr ? unsafe_goal->open_claw : false;
  if (unsafe_goal) {
    if (unsafe_goal->open_threshold != 0.0) {
      if (arm_.current_node() != unsafe_goal->arm_goal_position ||
          arm_.path_distance_to_go() > unsafe_goal->open_threshold) {
        open_claw = false;
      }
    }
  }
  arm_.Iterate(
      unsafe_goal != nullptr ? &(unsafe_goal->arm_goal_position) : nullptr,
      unsafe_goal != nullptr ? unsafe_goal->grab_box : false, open_claw,
      unsafe_goal != nullptr ? unsafe_goal->close_claw : false,
      &(position->arm), position->claw_beambreak_triggered,
      position->box_back_beambreak_triggered, intake_clear_of_box,
      unsafe_goal != nullptr ? unsafe_goal->voltage_winch > 1.0 : false,
      unsafe_goal != nullptr ? unsafe_goal->trajectory_override : false,
      output != nullptr ? &(output->voltage_proximal) : nullptr,
      output != nullptr ? &(output->voltage_distal) : nullptr,
      output != nullptr ? &(output->release_arm_brake) : nullptr,
      output != nullptr ? &(output->claw_grabbed) : nullptr, &(status->arm));

  if (output) {
    if (unsafe_goal) {
      output->hook_release = unsafe_goal->hook_release;
      output->voltage_winch = unsafe_goal->voltage_winch;
      output->forks_release = unsafe_goal->deploy_fork;
    } else {
      output->voltage_winch = 0.0;
      output->hook_release = false;
      output->forks_release = false;
    }
  }

  status->estopped = status->left_intake.estopped ||
                     status->right_intake.estopped || status->arm.estopped;

  status->zeroed = status->left_intake.zeroed && status->right_intake.zeroed &&
                   status->arm.zeroed;

  if (output && unsafe_goal) {
    double roller_voltage = ::std::max(
        -kMaxIntakeRollerVoltage, ::std::min(unsafe_goal->intake.roller_voltage,
                                             kMaxIntakeRollerVoltage));
    constexpr int kReverseTime = 14;
    if (unsafe_goal->intake.roller_voltage < 0.0 ||
        unsafe_goal->disable_box_correct) {
      output->left_intake.voltage_rollers = roller_voltage;
      output->right_intake.voltage_rollers = roller_voltage;
      rotation_state_ = RotationState::NOT_ROTATING;
      rotation_count_ = 0;
      stuck_count_ = 0;
    } else {
      monotonic_clock::time_point monotonic_now = monotonic_clock::now();
      const bool stuck = position->box_distance < 0.20 &&
                   filtered_box_velocity_ > -0.05 &&
                   !position->box_back_beambreak_triggered;
      // Make sure we don't declare ourselves re-stuck too quickly.  We want to
      // wait 400 ms before triggering the stuck condition again.
      if (!stuck) {
        last_unstuck_time_ = monotonic_now;
      }
      if (monotonic_now < last_stuck_time_ + chrono::milliseconds(400)) {
        last_unstuck_time_ = monotonic_now;
      }

      switch (rotation_state_) {
        case RotationState::NOT_ROTATING:
          if (stuck &&
              monotonic_now > last_stuck_time_ + chrono::milliseconds(400) &&
              monotonic_now > last_unstuck_time_ + chrono::milliseconds(100)) {
            rotation_state_ = RotationState::STUCK;
            ++stuck_count_;
            last_stuck_time_ = monotonic_now;
          } else if (position->left_intake.beam_break) {
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
        case RotationState::STUCK: {
          // Latch being stuck for 80 ms so we kick the box out far enough.
          if (last_stuck_time_ + chrono::milliseconds(80) < monotonic_now) {
            rotation_state_ = RotationState::NOT_ROTATING;
            last_unstuck_time_ = monotonic_now;
          }
        } break;
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

      constexpr double kHoldVoltage = 1.0;
      constexpr double kStuckVoltage = 10.0;

      if (position->box_back_beambreak_triggered &&
          roller_voltage > kHoldVoltage) {
        roller_voltage = kHoldVoltage;
      }
      switch (rotation_state_) {
        case RotationState::NOT_ROTATING: {
          double centering_gain = 13.0;
          if (stuck_count_ > 1) {
            if ((stuck_count_ - 1) % 2 == 0) {
              centering_gain = 0.0;
            }
          }
          output->left_intake.voltage_rollers =
              roller_voltage - intake_center_error * centering_gain;
          output->right_intake.voltage_rollers =
              roller_voltage + intake_center_error * centering_gain;
        } break;
        case RotationState::STUCK: {
          if (roller_voltage > kHoldVoltage) {
            output->left_intake.voltage_rollers = -kStuckVoltage;
            output->right_intake.voltage_rollers = -kStuckVoltage;
          }
        } break;
        case RotationState::ROTATING_LEFT:
          if (position->left_intake.beam_break) {
            output->left_intake.voltage_rollers = -roller_voltage * 0.9;
          } else {
            output->left_intake.voltage_rollers = -roller_voltage * 0.6;
          }
          output->right_intake.voltage_rollers = roller_voltage;
          break;
        case RotationState::ROTATING_RIGHT:
          output->left_intake.voltage_rollers = roller_voltage;
          if (position->right_intake.beam_break) {
            output->right_intake.voltage_rollers = -roller_voltage * 0.9;
          } else {
            output->right_intake.voltage_rollers = -roller_voltage * 0.6;
          }
          break;
      }
    }
  } else {
    rotation_state_ = RotationState::NOT_ROTATING;
    rotation_count_ = 0;
    stuck_count_ = 0;
  }
  status->rotation_state = static_cast<uint32_t>(rotation_state_);

  ::frc971::control_loops::drivetrain_queue.output.FetchLatest();

  ::y2018::vision::vision_status.FetchLatest();
  if (status->estopped) {
    SendColors(0.5, 0.0, 0.0);
  } else if (!y2018::vision::vision_status.get() ||
             y2018::vision::vision_status.Age() > chrono::seconds(1)) {
    SendColors(0.5, 0.5, 0.0);
  } else if (rotation_state_ == RotationState::ROTATING_LEFT ||
             rotation_state_ == RotationState::ROTATING_RIGHT) {
    SendColors(0.5, 0.20, 0.0);
  } else if (rotation_state_ == RotationState::STUCK) {
    SendColors(0.5, 0.0, 0.5);
  } else if (position->box_back_beambreak_triggered) {
    SendColors(0.0, 0.0, 0.5);
  } else if (position->box_distance < 0.2) {
    SendColors(0.0, 0.5, 0.0);
  } else if (::frc971::control_loops::drivetrain_queue.output.get() &&
             ::std::max(::std::abs(::frc971::control_loops::drivetrain_queue
                                       .output->left_voltage),
                        ::std::abs(::frc971::control_loops::drivetrain_queue
                                       .output->right_voltage)) > 11.5) {
    SendColors(0.5, 0.0, 0.5);
  } else {
    SendColors(0.0, 0.0, 0.0);
  }

  last_box_distance_ = clipped_box_distance;
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
