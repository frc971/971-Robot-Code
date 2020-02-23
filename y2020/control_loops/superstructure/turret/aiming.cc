#include "y2020/control_loops/superstructure/turret/aiming.h"

#include "y2020/control_loops/drivetrain/drivetrain_base.h"

namespace y2020 {
namespace control_loops {
namespace superstructure {
namespace turret {

using frc971::control_loops::Pose;

namespace {
// The overall length and width of the field, in meters.
constexpr double kFieldLength = 15.983;
constexpr double kFieldWidth = 8.212;
// Height of the center of the port(s) above the ground, in meters.
constexpr double kPortHeight = 2.494;

// Maximum shot angle at which we will attempt to make the shot into the inner
// port, in radians. Zero would imply that we could only shoot if we were
// exactly perpendicular to the target. Larger numbers allow us to aim at the
// inner port more aggressively, at the risk of being more likely to miss the
// outer port entirely.
constexpr double kMaxInnerPortAngle = 20.0 * M_PI / 180.0;

// Distance (in meters) from the edge of the field to the port.
constexpr double kEdgeOfFieldToPort = 2.404;

// The amount (in meters) that the inner port is set back from the outer port.
constexpr double kInnerPortBackset = 0.743;

// Minimum distance that we must be from the inner port in order to attempt the
// shot--this is to account for the fact that if we are too close to the target,
// then we won't have a clear shot on the inner port.
constexpr double kMinimumInnerPortShotDistance = 4.0;

Pose ReverseSideOfField(Pose target) {
  *target.mutable_pos() *= -1;
  target.set_theta(aos::math::NormalizeAngle(target.rel_theta() + M_PI));
  return target;
}

flatbuffers::DetachedBuffer MakePrefilledGoal() {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  Aimer::Goal::Builder builder(fbb);
  builder.add_unsafe_goal(0);
  builder.add_goal_velocity(0);
  builder.add_ignore_profile(true);
  fbb.Finish(builder.Finish());
  return fbb.Release();
}
}  // namespace

Pose InnerPortPose(aos::Alliance alliance) {
  const Pose target({kFieldLength / 2 + kInnerPortBackset,
                     -kFieldWidth / 2.0 + kEdgeOfFieldToPort, kPortHeight},
                    0.0);
  if (alliance == aos::Alliance::kRed) {
    return ReverseSideOfField(target);
  }
  return target;
}

Pose OuterPortPose(aos::Alliance alliance) {
  Pose target(
      {kFieldLength / 2, -kFieldWidth / 2.0 + kEdgeOfFieldToPort, kPortHeight},
      0.0);
  if (alliance == aos::Alliance::kRed) {
    return ReverseSideOfField(target);
  }
  return target;
}

Aimer::Aimer() : goal_(MakePrefilledGoal()) {}

void Aimer::Update(const Status *status, aos::Alliance alliance) {
  // This doesn't do anything intelligent with wrapping--it just produces a
  // result in the range (-pi, pi] rather than taking advantage of the turret's
  // full range.
  const Pose robot_pose({status->x(), status->y(), 0}, status->theta());
  const Pose inner_port = InnerPortPose(alliance);
  const Pose outer_port = OuterPortPose(alliance);
  const Pose robot_pose_from_inner_port = robot_pose.Rebase(&inner_port);
  const double inner_port_angle = robot_pose_from_inner_port.heading();
  const double inner_port_distance = robot_pose_from_inner_port.xy_norm();
  aiming_for_inner_port_ =
      (std::abs(inner_port_angle) < kMaxInnerPortAngle) &&
      (inner_port_distance > kMinimumInnerPortShotDistance);
  const Pose goal =
      (aiming_for_inner_port_ ? inner_port : outer_port).Rebase(&robot_pose);
  const double heading_to_goal = goal.heading();
  CHECK(status->has_localizer());
  distance_ = goal.xy_norm();
  // TODO(james): This code should probably just be in the localizer and have
  // xdot/ydot get populated in the status message directly... that way we don't
  // keep duplicating this math.
  // Also, this doesn't currently take into account the lateral velocity of the
  // robot. All of this would be helped by just doing this work in the Localizer
  // itself.
  const Eigen::Vector2d linear_angular =
      drivetrain::GetDrivetrainConfig().Tlr_to_la() *
      Eigen::Vector2d(status->localizer()->left_velocity(),
                      status->localizer()->right_velocity());
  // X and Y dot are negated because we are interested in the derivative of
  // (target_pos - robot_pos).
  const double xdot = -linear_angular(0) * std::cos(status->theta());
  const double ydot = -linear_angular(0) * std::sin(status->theta());
  const double rel_x = goal.rel_pos().x();
  const double rel_y = goal.rel_pos().y();
  const double squared_norm = rel_x * rel_x + rel_y * rel_y;
  // If squared_norm gets to be too close to zero, just zero out the relevant
  // term to prevent NaNs. Note that this doesn't address the chattering that
  // would likely occur if we were to get excessively close to the target.
  const double atan_diff = (squared_norm < 1e-3)
                               ? 0.0
                               : (rel_x * ydot - rel_y * xdot) / squared_norm;
  // heading = atan2(relative_y, relative_x) - robot_theta
  // dheading / dt = (rel_x * rel_y' - rel_y * rel_x') / (rel_x^2 + rel_y^2) - dtheta / dt
  const double dheading_dt = atan_diff - linear_angular(1);

  goal_.mutable_message()->mutate_unsafe_goal(heading_to_goal);
  goal_.mutable_message()->mutate_goal_velocity(dheading_dt);
}

flatbuffers::Offset<AimerStatus> Aimer::PopulateStatus(
    flatbuffers::FlatBufferBuilder *fbb) const {
  AimerStatus::Builder builder(*fbb);
  builder.add_turret_position(goal_.message().unsafe_goal());
  builder.add_turret_velocity(goal_.message().goal_velocity());
  builder.add_aiming_for_inner_port(aiming_for_inner_port_);
  return builder.Finish();
}

}  // namespace turret
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020
