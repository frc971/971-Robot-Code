#include "y2020/control_loops/superstructure/turret/aiming.h"

#include "y2020/constants.h"
#include "y2020/control_loops/drivetrain/drivetrain_base.h"

namespace y2020 {
namespace control_loops {
namespace superstructure {
namespace turret {

using frc971::control_loops::Pose;

// Shooting-on-the-fly concept:
// The current way that we manage shooting-on-the fly endeavors to be reasonably
// simple, until we get a chance to see how the actual dynamics play out.
// Essentially, we assume that the robot's velocity will represent a constant
// offset to the ball's velocity over the entire trajectory to the goal and
// then offset the target that we are pointing at based on that.
// Let us assume that, if the robot shoots while not moving, regardless of shot
// distance, the ball's average speed-over-ground to the target will be a
// constant s_shot (this implies that if the robot is driving straight towards
// the target, the actual ball speed-over-ground will be greater than s_shot).
// We will define things in the robot's coordinate frame. We will be shooting
// at a target that is at position (target_x, target_y) in the robot frame. The
// robot is travelling at (v_robot_x, v_robot_y). In order to shoot the ball,
// we need to generate some virtual target (virtual_x, virtual_y) that we will
// shoot at as if we were standing still. The total time-of-flight to that
// target will be t_shot = norm2(virtual_x, virtual_y) / s_shot.
// we will have virtual_x + v_robot_x * t_shot = target_x, and the same
// for y. This gives us three equations and three unknowns (virtual_x,
// virtual_y, and t_shot), and given appropriate assumptions, can be solved
// analytically. However, doing so is obnoxious and given appropriate functions
// for t_shot may not be feasible. As such, instead of actually solving the
// equation analytically, we will use an iterative solution where we maintain
// a current virtual target estimate. We start with this estimate as if the
// robot is stationary. We then use this estimate to calculate t_shot, and
// calculate the next value for the virtual target.

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
constexpr double kMaxInnerPortAngle = 10.0 * M_PI / 180.0;

// Distance (in meters) from the edge of the field to the port, with some
// compensation to ensure that our definition of where the target is matches
// that reported by the cameras.
constexpr double kEdgeOfFieldToPort = 2.404 + .0034;

// The amount (in meters) that the inner port is set back from the outer port.
constexpr double kInnerPortBackset = 0.743;

// Average speed-over-ground of the ball on its way to the target. Our current
// model assumes constant ball velocity regardless of shot distance.
// TODO(james): Is this an appropriate model? For the outer port it should be
// good enough that it doesn't really matter, but for the inner port it may be
// more appropriate to do something more dynamic--however, it is not yet clear
// how we would best estimate speed-over-ground given a hood angle + shooter
// speed. Assuming a constant average speed over the course of the trajectory
// should be reasonable, since all we are trying to do here is calculate an
// overall time-of-flight (we don't actually care about the ball speed itself).
constexpr double kBallSpeedOverGround = 15.0;  // m/s

// Minimum distance that we must be from the inner port in order to attempt the
// shot--this is to account for the fact that if we are too close to the target,
// then we won't have a clear shot on the inner port.
constexpr double kMinimumInnerPortShotDistance = 3.9;

// Amount of buffer, in radians, to leave to help avoid wrapping. I.e., any time
// that we are in kAvoidEdges mode, we will keep ourselves at least
// kAntiWrapBuffer radians away from the hardstops.
constexpr double kAntiWrapBuffer = 0.2;

// If the turret is at zero, then it will be at this angle relative to pointed
// straight forwards on the robot.
constexpr double kTurretZeroOffset = M_PI;

constexpr double kTurretRange = constants::Values::kTurretRange().range();
static_assert((kTurretRange - 2.0 * kAntiWrapBuffer) > 2.0 * M_PI,
              "kAntiWrap buffer should be small enough that we still have 360 "
              "degrees of range.");

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

// This implements the iteration in the described shooting-on-the-fly algorithm.
// robot_pose: Current robot pose.
// robot_velocity: Current robot velocity, in the absolute field frame.
// target_pose: Absolute goal Pose.
// current_virtual_pose: Current estimate of where we want to shoot at.
Pose IterateVirtualGoal(const Pose &robot_pose,
                        const Eigen::Vector3d &robot_velocity,
                        const Pose &target_pose,
                        const Pose &current_virtual_pose) {
  const double air_time =
      current_virtual_pose.Rebase(&robot_pose).xy_norm() / kBallSpeedOverGround;
  const Eigen::Vector3d virtual_target =
      target_pose.abs_pos() - air_time * robot_velocity;
  return Pose(virtual_target, target_pose.abs_theta());
}
}  // namespace

Pose InnerPortPose(aos::Alliance alliance) {
  const Pose target({kFieldLength / 2 + kInnerPortBackset,
                     -kFieldWidth / 2.0 + kEdgeOfFieldToPort, kPortHeight},
                    M_PI);
  if (alliance == aos::Alliance::kRed) {
    return ReverseSideOfField(target);
  }
  return target;
}

Pose OuterPortPose(aos::Alliance alliance) {
  Pose target(
      {kFieldLength / 2, -kFieldWidth / 2.0 + kEdgeOfFieldToPort, kPortHeight},
      M_PI);
  if (alliance == aos::Alliance::kRed) {
    return ReverseSideOfField(target);
  }
  return target;
}

Aimer::Aimer() : goal_(MakePrefilledGoal()) {}

void Aimer::Update(const Status *status, aos::Alliance alliance,
                   WrapMode wrap_mode, ShotMode shot_mode) {
  const Pose robot_pose({status->x(), status->y(), 0}, status->theta());
  const Pose inner_port = InnerPortPose(alliance);
  const Pose outer_port = OuterPortPose(alliance);
  const Pose robot_pose_from_inner_port = robot_pose.Rebase(&inner_port);

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
  const double xdot = linear_angular(0) * std::cos(status->theta());
  const double ydot = linear_angular(0) * std::sin(status->theta());

  inner_port_angle_ = robot_pose_from_inner_port.heading();
  const double inner_port_distance = robot_pose_from_inner_port.rel_pos().x();
  // Add a bit of hysteresis so that we don't jump between aiming for the inner
  // and outer ports.
  const double max_inner_port_angle =
      aiming_for_inner_port_ ? 1.2 * kMaxInnerPortAngle : kMaxInnerPortAngle;
  const double min_inner_port_distance =
      aiming_for_inner_port_ ? (kMinimumInnerPortShotDistance - 0.3)
                             : kMinimumInnerPortShotDistance;
  aiming_for_inner_port_ =
      (std::abs(inner_port_angle_) < max_inner_port_angle) &&
      (inner_port_distance > min_inner_port_distance);

  // This code manages compensating the goal turret heading for the robot's
  // current velocity, to allow for shooting on-the-fly.
  // This works by solving for the correct turret angle numerically, since while
  // we technically could do it analytically, doing so would both make it hard
  // to make small changes (since it would force us to redo the math) and be
  // error-prone since it'd be easy to make typos or other minor math errors.
  Pose virtual_goal;
  {
    const Pose goal = aiming_for_inner_port_ ? inner_port : outer_port;
    target_distance_ = goal.Rebase(&robot_pose).xy_norm();
    virtual_goal = goal;
    if (shot_mode == ShotMode::kShootOnTheFly) {
      for (int ii = 0; ii < 3; ++ii) {
        virtual_goal =
            IterateVirtualGoal(robot_pose, {xdot, ydot, 0}, goal, virtual_goal);
      }
      VLOG(1) << "Shooting-on-the-fly target position: "
              << virtual_goal.abs_pos().transpose();
    }
    virtual_goal = virtual_goal.Rebase(&robot_pose);
  }

  const double heading_to_goal = virtual_goal.heading();
  CHECK(status->has_localizer());
  shot_distance_ = virtual_goal.xy_norm();

  // The following code all works to calculate what the rate of turn of the
  // turret should be. The code only accounts for the rate of turn if we are
  // aiming at a static target, which should be close enough to correct that it
  // doesn't matter that it fails to account for the
  // shooting-on-the-fly compensation.
  const double rel_x = virtual_goal.rel_pos().x();
  const double rel_y = virtual_goal.rel_pos().y();
  const double squared_norm = rel_x * rel_x + rel_y * rel_y;
  // rel_xdot and rel_ydot are the derivatives (with respect to time) of rel_x
  // and rel_y. Since these are in the robot's coordinate frame, and since we
  // are ignoring lateral velocity for this exercise, rel_ydot is zero, and
  // rel_xdot is just the inverse of the robot's velocity.
  const double rel_xdot = -linear_angular(0);
  const double rel_ydot = 0.0;

  // If squared_norm gets to be too close to zero, just zero out the relevant
  // term to prevent NaNs. Note that this doesn't address the chattering that
  // would likely occur if we were to get excessively close to the target.
  // Note that x and y terms are swapped relative to what you would normally see
  // in the derivative of atan because xdot and ydot are the derivatives of
  // robot_pos and we are working with the atan of (target_pos - robot_pos).
  const double atan_diff =
      (squared_norm < 1e-3) ? 0.0 : (rel_x * rel_ydot - rel_y * rel_xdot) /
                                        squared_norm;
  // heading = atan2(relative_y, relative_x) - robot_theta
  // dheading / dt =
  //     (rel_x * rel_y' - rel_y * rel_x') / (rel_x^2 + rel_y^2) - dtheta / dt
  const double dheading_dt = atan_diff - linear_angular(1);

  double range = kTurretRange;
  if (wrap_mode == WrapMode::kAvoidEdges) {
    range -= 2.0 * kAntiWrapBuffer;
  }
  // Calculate a goal turret heading such that it is within +/- pi of the
  // current position (i.e., a goal that would minimize the amount the turret
  // would have to travel).
  // We then check if this goal would bring us out of range of the valid angles,
  // and if it would, we reset to be within +/- pi of zero.
  double turret_heading =
      goal_.message().unsafe_goal() +
      aos::math::NormalizeAngle(heading_to_goal - kTurretZeroOffset -
                                goal_.message().unsafe_goal());
  if (std::abs(turret_heading - constants::Values::kTurretRange().middle()) >
      range / 2.0) {
    turret_heading = aos::math::NormalizeAngle(turret_heading);
  }

  goal_.mutable_message()->mutate_unsafe_goal(turret_heading);
  goal_.mutable_message()->mutate_goal_velocity(
      std::clamp(dheading_dt, -2.0, 2.0));
}

flatbuffers::Offset<AimerStatus> Aimer::PopulateStatus(
    flatbuffers::FlatBufferBuilder *fbb) const {
  AimerStatus::Builder builder(*fbb);
  builder.add_turret_position(goal_.message().unsafe_goal());
  builder.add_turret_velocity(goal_.message().goal_velocity());
  builder.add_aiming_for_inner_port(aiming_for_inner_port_);
  builder.add_target_distance(target_distance_);
  builder.add_inner_port_angle(inner_port_angle_);
  builder.add_shot_distance(DistanceToGoal());
  return builder.Finish();
}

}  // namespace turret
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020
