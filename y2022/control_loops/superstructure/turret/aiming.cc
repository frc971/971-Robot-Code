#include "y2022/control_loops/superstructure/turret/aiming.h"

#include "y2022/control_loops/drivetrain/drivetrain_base.h"

namespace y2022 {
namespace control_loops {
namespace superstructure {
namespace turret {

using frc971::control_loops::Pose;
using frc971::control_loops::aiming::RobotState;
using frc971::control_loops::aiming::ShotConfig;

namespace {
// If the turret is at zero, then it will be at this angle at which the shot
// will leave the robot. I.e., if the turret is at zero, then the shot will go
// straight out the back of the robot.
constexpr double kTurretZeroOffset = M_PI;

constexpr double kMaxProfiledVelocity = 10.0;
constexpr double kMaxProfiledAccel = 25.0;

flatbuffers::DetachedBuffer MakePrefilledGoal() {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  frc971::ProfileParameters::Builder profile_builder(fbb);
  profile_builder.add_max_velocity(kMaxProfiledVelocity);
  profile_builder.add_max_acceleration(kMaxProfiledAccel);
  const flatbuffers::Offset<frc971::ProfileParameters> profile_offset =
      profile_builder.Finish();
  Aimer::Goal::Builder builder(fbb);
  builder.add_unsafe_goal(0);
  builder.add_goal_velocity(0);
  builder.add_ignore_profile(false);
  builder.add_profile_params(profile_offset);
  fbb.Finish(builder.Finish());
  return fbb.Release();
}
}  // namespace

Aimer::Aimer(std::shared_ptr<const constants::Values> constants)
    : constants_(constants), goal_(MakePrefilledGoal()) {}

void Aimer::Update(const Status *status, ShotMode shot_mode) {
  const Pose robot_pose({status->x(), status->y(), 0}, status->theta());
  const Pose goal({0.0, 0.0, 0.0}, 0.0);

  const Eigen::Vector2d linear_angular =
      drivetrain::GetDrivetrainConfig().Tlr_to_la() *
      Eigen::Vector2d(status->estimated_left_velocity(),
                      status->estimated_right_velocity());
  const double xdot = linear_angular(0) * std::cos(status->theta());
  const double ydot = linear_angular(0) * std::sin(status->theta());

  // Use the previous shot distance to estimate the speed-over-ground of the
  // ball.
  current_goal_ = frc971::control_loops::aiming::AimerGoal(
      ShotConfig{goal, shot_mode, constants_->turret_range,
                 constants_->shot_velocity_interpolation_table
                     .Get(current_goal_.target_distance)
                     .shot_speed_over_ground,
                 /*wrap_mode=*/0.0, kTurretZeroOffset},
      RobotState{robot_pose,
                 {xdot, ydot},
                 linear_angular(1),
                 goal_.message().unsafe_goal()});

  goal_.mutable_message()->mutate_unsafe_goal(current_goal_.position);
  goal_.mutable_message()->mutate_goal_velocity(
      std::clamp(current_goal_.velocity, 0.0, 0.0));
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

}  // namespace turret
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022
