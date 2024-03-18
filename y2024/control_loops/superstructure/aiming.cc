#include "y2024/control_loops/superstructure/aiming.h"

#include "frc971/control_loops/aiming/aiming.h"
#include "frc971/control_loops/pose.h"

using frc971::control_loops::aiming::RobotState;
using frc971::control_loops::aiming::ShotConfig;
using frc971::control_loops::aiming::ShotMode;
using y2024::control_loops::superstructure::Aimer;

Aimer::Aimer(aos::EventLoop *event_loop,
             const y2024::Constants *robot_constants)
    : event_loop_(event_loop),
      robot_constants_(CHECK_NOTNULL(robot_constants)),
      drivetrain_config_(
          frc971::control_loops::drivetrain::DrivetrainConfig<double>::
              FromFlatbuffer(*robot_constants_->common()->drivetrain())),
      interpolation_table_(
          y2024::constants::Values::InterpolationTableFromFlatbuffer(
              robot_constants_->common()->shooter_interpolation_table())),
      joystick_state_fetcher_(
          event_loop_->MakeFetcher<aos::JoystickState>("/aos")) {}

void Aimer::Update(
    const frc971::control_loops::drivetrain::Status *status,
    frc971::control_loops::aiming::ShotMode shot_mode,
    frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoalStatic
        *turret_goal) {
  if (status == nullptr) {
    return;
  }
  const frc971::control_loops::Pose robot_pose({status->x(), status->y(), 0},
                                               status->theta());
  aos::Alliance alliance = aos::Alliance::kRed;
  if (!joystick_state_fetcher_.Fetch() && !received_joystick_state_) {
    received_joystick_state_ = false;
  } else {
    received_joystick_state_ = true;

    CHECK_NOTNULL(joystick_state_fetcher_.get());
    alliance = joystick_state_fetcher_->alliance();
  }

  const frc971::control_loops::Pose red_alliance_goal(
      frc971::ToEigenOrDie<3, 1>(*robot_constants_->common()
                                      ->shooter_targets()
                                      ->red_alliance()
                                      ->pos()),
      robot_constants_->common()->shooter_targets()->red_alliance()->theta());

  const frc971::control_loops::Pose blue_alliance_goal(
      frc971::ToEigenOrDie<3, 1>(*robot_constants_->common()
                                      ->shooter_targets()
                                      ->blue_alliance()
                                      ->pos()),
      robot_constants_->common()->shooter_targets()->blue_alliance()->theta());

  const frc971::control_loops::Pose goal =
      alliance == aos::Alliance::kRed ? red_alliance_goal : blue_alliance_goal;

  const Eigen::Vector2d linear_angular =
      drivetrain_config_.Tlr_to_la() *
      Eigen::Vector2d(status->estimated_left_velocity(),
                      status->estimated_right_velocity());
  const double xdot = linear_angular(0) * std::cos(status->theta());
  const double ydot = linear_angular(0) * std::sin(status->theta());

  // Use the previous shot distance to estimate the speed-over-ground of the
  // note.
  current_goal_ = frc971::control_loops::aiming::AimerGoal(
      ShotConfig{goal, shot_mode,
                 frc971::constants::Range::FromFlatbuffer(
                     robot_constants_->common()->turret()->range()),
                 interpolation_table_.Get(current_goal_.target_distance)
                     .shot_speed_over_ground,
                 /*wrap_mode=*/0.15, M_PI - kTurretZeroOffset},
      RobotState{
          robot_pose, {xdot, ydot}, linear_angular(1), current_goal_.position});

  turret_goal->set_unsafe_goal(current_goal_.position);
}

flatbuffers::Offset<AimerStatus> Aimer::PopulateStatus(
    flatbuffers::FlatBufferBuilder *fbb) const {
  AimerStatus::Builder builder(*fbb);
  builder.add_turret_position(current_goal_.position);
  builder.add_turret_velocity(current_goal_.velocity);
  builder.add_target_distance(current_goal_.target_distance);
  builder.add_shot_distance(DistanceToGoal());
  return builder.Finish();
}
