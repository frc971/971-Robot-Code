#include "y2018/control_loops/superstructure/superstructure.h"

#include <chrono>

#include "aos/logging/logging.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "y2018/constants.h"
#include "y2018/control_loops/superstructure/intake/intake.h"
#include "y2018/status_light_generated.h"
#include "y2018/vision/vision_generated.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {

using ::aos::monotonic_clock;

namespace chrono = ::std::chrono;

namespace {
// The maximum voltage the intake roller will be allowed to use.
constexpr double kMaxIntakeRollerVoltage = 12.0;
}  // namespace

Superstructure::Superstructure(::aos::EventLoop *event_loop,
                               const ::std::string &name)
    : aos::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                 name),
      status_light_sender_(
          event_loop->MakeSender<::y2018::StatusLight>("/superstructure")),
      vision_status_fetcher_(
          event_loop->MakeFetcher<::y2018::vision::VisionStatus>(
              "/superstructure")),
      drivetrain_output_fetcher_(
          event_loop->MakeFetcher<::frc971::control_loops::drivetrain::Output>(
              "/drivetrain")),
      intake_left_(constants::GetValues().left_intake.zeroing,
                   constants::GetValues().left_intake.spring_offset),
      intake_right_(constants::GetValues().right_intake.zeroing,
                    constants::GetValues().right_intake.spring_offset) {}

void Superstructure::RunIteration(const Goal *unsafe_goal,
                                  const Position *position,
                                  aos::Sender<Output>::Builder *output,
                                  aos::Sender<Status>::Builder *status) {
  const monotonic_clock::time_point monotonic_now =
      event_loop()->monotonic_now();
  if (WasReset()) {
    AOS_LOG(ERROR, "WPILib reset, restarting\n");
    intake_left_.Reset();
    intake_right_.Reset();
    arm_.Reset();
  }

  const double clipped_box_distance =
      ::std::min(1.0, ::std::max(0.0, position->box_distance()));

  const double box_velocity =
      (clipped_box_distance - last_box_distance_) / 0.005;

  constexpr double kFilteredBoxVelocityAlpha = 0.02;
  filtered_box_velocity_ =
      box_velocity * kFilteredBoxVelocityAlpha +
      (1.0 - kFilteredBoxVelocityAlpha) * filtered_box_velocity_;

  constexpr double kCenteringAngleGain = 0.0;
  const double left_intake_goal =
      ::std::min(arm_.max_intake_override(),
                 (unsafe_goal == nullptr || !unsafe_goal->has_intake()
                      ? 0.0
                      : unsafe_goal->intake()->left_intake_angle())) +
      last_intake_center_error_ * kCenteringAngleGain;
  const double right_intake_goal =
      ::std::min(arm_.max_intake_override(),
                 (unsafe_goal == nullptr || !unsafe_goal->has_intake()
                      ? 0.0
                      : unsafe_goal->intake()->right_intake_angle())) -
      last_intake_center_error_ * kCenteringAngleGain;

  IntakeVoltageT left_intake_output;
  flatbuffers::Offset<superstructure::IntakeSideStatus> left_status_offset =
      intake_left_.Iterate(
          unsafe_goal != nullptr ? &(left_intake_goal) : nullptr,
          position->left_intake(),
          output != nullptr ? &(left_intake_output) : nullptr, status->fbb());

  IntakeVoltageT right_intake_output;
  flatbuffers::Offset<superstructure::IntakeSideStatus> right_status_offset =
      intake_right_.Iterate(
          unsafe_goal != nullptr ? &(right_intake_goal) : nullptr,
          position->right_intake(),
          output != nullptr ? &(right_intake_output) : nullptr, status->fbb());

  const double intake_center_error =
      intake_right_.output_position() - intake_left_.output_position();
  last_intake_center_error_ = intake_center_error;

  const bool intake_clear_of_box =
      intake_left_.clear_of_box() && intake_right_.clear_of_box();

  bool open_claw = unsafe_goal != nullptr ? unsafe_goal->open_claw() : false;
  if (unsafe_goal) {
    if (unsafe_goal->open_threshold() != 0.0) {
      if (arm_.current_node() != unsafe_goal->arm_goal_position() ||
          arm_.path_distance_to_go() > unsafe_goal->open_threshold()) {
        open_claw = false;
      }
    }
  }

  const uint32_t arm_goal_position =
      unsafe_goal != nullptr ? unsafe_goal->arm_goal_position() : 0u;

  double voltage_proximal_output = 0.0;
  double voltage_distal_output = 0.0;
  bool release_arm_brake_output = false;
  bool claw_grabbed_output = false;
  flatbuffers::Offset<superstructure::ArmStatus> arm_status_offset =
      arm_.Iterate(
          monotonic_now,
          unsafe_goal != nullptr ? &(arm_goal_position) : nullptr,
          unsafe_goal != nullptr ? unsafe_goal->grab_box() : false, open_claw,
          unsafe_goal != nullptr ? unsafe_goal->close_claw() : false,
          position->arm(), position->claw_beambreak_triggered(),
          position->box_back_beambreak_triggered(), intake_clear_of_box,
          unsafe_goal != nullptr ? unsafe_goal->voltage_winch() > 1.0 : false,
          unsafe_goal != nullptr ? unsafe_goal->trajectory_override() : false,
          output != nullptr ? &voltage_proximal_output : nullptr,
          output != nullptr ? &voltage_distal_output : nullptr,
          output != nullptr ? &release_arm_brake_output : nullptr,
          output != nullptr ? &claw_grabbed_output : nullptr, status->fbb());


  bool hook_release_output = false;
  bool forks_release_output = false;
  double voltage_winch_output = 0.0;
  if (output) {
    if (unsafe_goal) {
      hook_release_output = unsafe_goal->hook_release();
      voltage_winch_output = unsafe_goal->voltage_winch();
      forks_release_output = unsafe_goal->deploy_fork();
    }
  }

  Status::Builder status_builder = status->MakeBuilder<Status>();

  status_builder.add_left_intake(left_status_offset);
  status_builder.add_right_intake(right_status_offset);
  status_builder.add_arm(arm_status_offset);

  status_builder.add_filtered_box_velocity(filtered_box_velocity_);
  const bool estopped =
      intake_left_.estopped() || intake_right_.estopped() || arm_.estopped();
  status_builder.add_estopped(estopped);

  status_builder.add_zeroed(intake_left_.zeroed() && intake_right_.zeroed() &&
                            arm_.zeroed());

  if (output && unsafe_goal) {
    double roller_voltage =
        ::std::max(-kMaxIntakeRollerVoltage,
                   ::std::min(unsafe_goal->intake()->roller_voltage(),
                              kMaxIntakeRollerVoltage));
    constexpr int kReverseTime = 14;
    if (unsafe_goal->intake()->roller_voltage() < 0.0 ||
        unsafe_goal->disable_box_correct()) {
      left_intake_output.voltage_rollers = roller_voltage;
      right_intake_output.voltage_rollers = roller_voltage;
      rotation_state_ = RotationState::NOT_ROTATING;
      rotation_count_ = 0;
      stuck_count_ = 0;
    } else {
      const bool stuck = position->box_distance() < 0.20 &&
                         filtered_box_velocity_ > -0.05 &&
                         !position->box_back_beambreak_triggered();
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
          } else if (position->left_intake()->beam_break()) {
            rotation_state_ = RotationState::ROTATING_RIGHT;
            rotation_count_ = kReverseTime;
            break;
          } else if (position->right_intake()->beam_break()) {
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
          if (position->right_intake()->beam_break()) {
            rotation_count_ = kReverseTime;
          } else {
            --rotation_count_;
          }
          if (rotation_count_ == 0) {
            rotation_state_ = RotationState::NOT_ROTATING;
          }
          break;
        case RotationState::ROTATING_RIGHT:
          if (position->left_intake()->beam_break()) {
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

      if (position->box_back_beambreak_triggered() &&
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
          left_intake_output.voltage_rollers =
              roller_voltage - intake_center_error * centering_gain;
          right_intake_output.voltage_rollers =
              roller_voltage + intake_center_error * centering_gain;
        } break;
        case RotationState::STUCK: {
          if (roller_voltage > kHoldVoltage) {
            left_intake_output.voltage_rollers = -kStuckVoltage;
            right_intake_output.voltage_rollers = -kStuckVoltage;
          }
        } break;
        case RotationState::ROTATING_LEFT:
          if (position->left_intake()->beam_break()) {
            left_intake_output.voltage_rollers = -roller_voltage * 0.9;
          } else {
            left_intake_output.voltage_rollers = -roller_voltage * 0.6;
          }
          right_intake_output.voltage_rollers = roller_voltage;
          break;
        case RotationState::ROTATING_RIGHT:
          left_intake_output.voltage_rollers = roller_voltage;
          if (position->right_intake()->beam_break()) {
            right_intake_output.voltage_rollers = -roller_voltage * 0.9;
          } else {
            right_intake_output.voltage_rollers = -roller_voltage * 0.6;
          }
          break;
      }
    }
  } else {
    rotation_state_ = RotationState::NOT_ROTATING;
    rotation_count_ = 0;
    stuck_count_ = 0;
  }
  status_builder.add_rotation_state(static_cast<uint32_t>(rotation_state_));

  drivetrain_output_fetcher_.Fetch();

  vision_status_fetcher_.Fetch();
  if (estopped) {
    SendColors(0.5, 0.0, 0.0);
  } else if (!vision_status_fetcher_.get() ||
             monotonic_now >
                 vision_status_fetcher_.context().monotonic_event_time +
                     chrono::seconds(1)) {
    SendColors(0.5, 0.5, 0.0);
  } else if (rotation_state_ == RotationState::ROTATING_LEFT ||
             rotation_state_ == RotationState::ROTATING_RIGHT) {
    SendColors(0.5, 0.20, 0.0);
  } else if (rotation_state_ == RotationState::STUCK) {
    SendColors(0.5, 0.0, 0.5);
  } else if (position->box_back_beambreak_triggered()) {
    SendColors(0.0, 0.0, 0.5);
  } else if (position->box_distance() < 0.2) {
    SendColors(0.0, 0.5, 0.0);
  } else if (drivetrain_output_fetcher_.get() &&
             ::std::max(
                 ::std::abs(drivetrain_output_fetcher_->left_voltage()),
                 ::std::abs(drivetrain_output_fetcher_->right_voltage())) >
                 11.5) {
    SendColors(0.5, 0.0, 0.5);
  } else {
    SendColors(0.0, 0.0, 0.0);
  }

  last_box_distance_ = clipped_box_distance;

  if (output) {
    flatbuffers::Offset<IntakeVoltage> left_intake_offset =
        IntakeVoltage::Pack(*output->fbb(), &left_intake_output);
    flatbuffers::Offset<IntakeVoltage> right_intake_offset =
        IntakeVoltage::Pack(*output->fbb(), &right_intake_output);

    Output::Builder output_builder = output->MakeBuilder<Output>();
    output_builder.add_left_intake(left_intake_offset);
    output_builder.add_right_intake(right_intake_offset);
    output_builder.add_voltage_proximal(voltage_proximal_output);
    output_builder.add_voltage_distal(voltage_distal_output);
    output_builder.add_release_arm_brake(release_arm_brake_output);
    output_builder.add_claw_grabbed(claw_grabbed_output);

    output_builder.add_hook_release(hook_release_output);
    output_builder.add_forks_release(forks_release_output);
    output_builder.add_voltage_winch(voltage_winch_output);

    output->Send(output_builder.Finish());
  }

  status->Send(status_builder.Finish());
}

void Superstructure::SendColors(float red, float green, float blue) {
  auto builder = status_light_sender_.MakeBuilder();
  StatusLight::Builder status_light_builder =
      builder.MakeBuilder<StatusLight>();

  status_light_builder.add_red(red);
  status_light_builder.add_green(green);
  status_light_builder.add_blue(blue);

  if (!builder.Send(status_light_builder.Finish())) {
    AOS_LOG(ERROR, "Failed to send lights.\n");
  }
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
