#include "frc971/control_loops/aiming/aiming.h"

#include "glog/logging.h"

namespace frc971::control_loops::aiming {

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
// This implements the iteration in the described shooting-on-the-fly algorithm.
// robot_pose: Current robot pose.
// robot_velocity: Current robot velocity, in the absolute field frame.
// target_pose: Absolute goal Pose.
// current_virtual_pose: Current estimate of where we want to shoot at.
// ball_speed_over_ground: Approximate ground speed of the ball that we are
// shooting.
Pose IterateVirtualGoal(const Pose &robot_pose,
                        const Eigen::Vector3d &robot_velocity,
                        const Pose &target_pose,
                        const Pose &current_virtual_pose,
                        double ball_speed_over_ground) {
  const double air_time = current_virtual_pose.Rebase(&robot_pose).xy_norm() /
                          ball_speed_over_ground;
  const Eigen::Vector3d virtual_target =
      target_pose.abs_pos() - air_time * robot_velocity;
  return Pose(virtual_target, target_pose.abs_theta());
}
}  // namespace

TurretGoal AimerGoal(const ShotConfig &config, const RobotState &state) {
  TurretGoal result;
  // This code manages compensating the goal turret heading for the robot's
  // current velocity, to allow for shooting on-the-fly.
  // This works by solving for the correct turret angle numerically, since while
  // we technically could do it analytically, doing so would both make it hard
  // to make small changes (since it would force us to redo the math) and be
  // error-prone since it'd be easy to make typos or other minor math errors.
  Pose virtual_goal;
  {
    result.target_distance = config.goal.Rebase(&state.pose).xy_norm();
    virtual_goal = config.goal;
    if (config.mode == ShotMode::kShootOnTheFly) {
      for (int ii = 0; ii < 3; ++ii) {
        virtual_goal = IterateVirtualGoal(
            state.pose, {state.velocity(0), state.velocity(1), 0}, config.goal,
            virtual_goal, config.ball_speed_over_ground);
      }
      VLOG(1) << "Shooting-on-the-fly target position: "
              << virtual_goal.abs_pos().transpose();
    }
    virtual_goal = virtual_goal.Rebase(&state.pose);
  }

  const double heading_to_goal = virtual_goal.heading();
  result.virtual_shot_distance = virtual_goal.xy_norm();

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
  // Note that rel_x and rel_y are in the robot frame.
  const double rel_xdot = -Eigen::Vector2d(std::cos(state.pose.rel_theta()),
                                           std::sin(state.pose.rel_theta()))
                               .dot(state.velocity);
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
  const double dheading_dt = atan_diff - state.yaw_rate;

  const double range =
      config.turret_range.range() - config.anti_wrap_buffer * 2.0;
  // Calculate a goal turret heading such that it is within +/- pi of the
  // current position (i.e., a goal that would minimize the amount the turret
  // would have to travel).
  // We then check if this goal would bring us out of range of the valid angles,
  // and if it would, we reset to be within +/- pi of zero.
  double turret_heading =
      state.last_turret_goal +
      aos::math::NormalizeAngle(heading_to_goal - config.turret_zero_offset -
                                state.last_turret_goal);
  if (std::abs(turret_heading - config.turret_range.middle()) > range / 2.0) {
    turret_heading = aos::math::NormalizeAngle(turret_heading);
  }
  result.position = turret_heading;
  result.velocity = dheading_dt;
  return result;
}

}  // namespace frc971::control_loops::aiming
