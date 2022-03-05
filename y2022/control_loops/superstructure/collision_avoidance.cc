#include "y2022/control_loops/superstructure/collision_avoidance.h"

#include <cmath>

#include "absl/functional/bind_front.h"
#include "glog/logging.h"

namespace y2022 {
namespace control_loops {
namespace superstructure {

CollisionAvoidance::CollisionAvoidance() {
  clear_min_intake_front_goal();
  clear_max_intake_front_goal();
  clear_min_intake_back_goal();
  clear_max_intake_back_goal();
  clear_min_turret_goal();
  clear_max_turret_goal();
}

bool CollisionAvoidance::IsCollided(const CollisionAvoidance::Status &status) {
  // Checks if intake front is collided.
  if (TurretCollided(status.intake_front_position, status.turret_position,
                     kMinCollisionZoneFrontTurret,
                     kMaxCollisionZoneFrontTurret)) {
    return true;
  }

  // Checks if intake back is collided.
  if (TurretCollided(status.intake_back_position, status.turret_position,
                     kMinCollisionZoneBackTurret,
                     kMaxCollisionZoneBackTurret)) {
    return true;
  }

  return false;
}

std::pair<double, int> WrapTurretAngle(double turret_angle) {
  double wrapped = std::remainder(turret_angle - M_PI, 2 * M_PI) + M_PI;
  int wraps =
      static_cast<int>(std::round((turret_angle - wrapped) / (2 * M_PI)));
  return {wrapped, wraps};
}

double UnwrapTurretAngle(double wrapped, int wraps) {
  return wrapped + 2.0 * M_PI * wraps;
}

bool CollisionAvoidance::TurretCollided(double intake_position,
                                        double turret_position,
                                        double min_turret_collision_position,
                                        double max_turret_collision_position) {
  const auto turret_position_wrapped_pair = WrapTurretAngle(turret_position);
  const double turret_position_wrapped = turret_position_wrapped_pair.first;

  // Checks if turret is in the collision area.
  if (turret_position_wrapped >= min_turret_collision_position &&
      turret_position_wrapped <= max_turret_collision_position) {
    // Reterns true if the intake is raised.
    if (intake_position <= kCollisionZoneIntake) {
      return true;
    }
  } else {
    return false;
  }
  return false;
}

void CollisionAvoidance::UpdateGoal(
    const CollisionAvoidance::Status &status,
    const frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal
        *unsafe_turret_goal) {
  // Start with our constraints being wide open.
  clear_max_turret_goal();
  clear_min_turret_goal();
  clear_max_intake_front_goal();
  clear_min_intake_front_goal();
  clear_max_intake_back_goal();
  clear_min_intake_back_goal();

  const double intake_front_position = status.intake_front_position;
  const double intake_back_position = status.intake_back_position;
  const double turret_position = status.turret_position;

  const double turret_goal = (unsafe_turret_goal != nullptr
                                  ? unsafe_turret_goal->unsafe_goal()
                                  : std::numeric_limits<double>::quiet_NaN());

  // Calculating the avoidance with either intake, and when the turret is
  // wrapped.

  CalculateAvoidance(true, intake_front_position, turret_goal,
                     kMinCollisionZoneFrontTurret, kMaxCollisionZoneFrontTurret,
                     turret_position);
  CalculateAvoidance(false, intake_back_position, turret_goal,
                     kMinCollisionZoneBackTurret, kMaxCollisionZoneBackTurret,
                     turret_position);
}

void CollisionAvoidance::CalculateAvoidance(bool intake_front,
                                            double intake_position,
                                            double turret_goal,
                                            double min_turret_collision_goal,
                                            double max_turret_collision_goal,
                                            double turret_position) {
  auto [turret_position_wrapped, turret_position_wraps] =
      WrapTurretAngle(turret_position);

  // If the turret goal is in a collison zone or moving through one, limit
  // intake.
  const bool turret_pos_unsafe =
      (turret_position_wrapped >= min_turret_collision_goal &&
       turret_position_wrapped <= max_turret_collision_goal);

  const bool turret_moving_forward = (turret_goal > turret_position);

  // To figure out if we are moving past an intake, find the unwrapped min/max
  // angles closest to the turret position on the journey.
  int bounds_wraps = turret_position_wraps;
  double min_turret_collision_goal_unwrapped =
      UnwrapTurretAngle(min_turret_collision_goal, bounds_wraps);
  if (turret_moving_forward &&
      min_turret_collision_goal_unwrapped < turret_position) {
    bounds_wraps++;
  } else if (!turret_moving_forward &&
             min_turret_collision_goal_unwrapped > turret_position) {
    bounds_wraps--;
  }
  min_turret_collision_goal_unwrapped =
      UnwrapTurretAngle(min_turret_collision_goal, bounds_wraps);
  const double max_turret_collision_goal_unwrapped =
      UnwrapTurretAngle(max_turret_collision_goal, bounds_wraps);

  // Check if the closest unwrapped angles are going to be passed
  const bool turret_moving_past_intake =
      ((turret_moving_forward &&
        (turret_position <= max_turret_collision_goal_unwrapped &&
         turret_goal >= min_turret_collision_goal_unwrapped)) ||
       (!turret_moving_forward &&
        (turret_position >= min_turret_collision_goal_unwrapped &&
         turret_goal <= max_turret_collision_goal_unwrapped)));

  if (turret_pos_unsafe || turret_moving_past_intake) {
    // If the turret is unsafe, limit the intake
    if (intake_front) {
      update_min_intake_front_goal(kCollisionZoneIntake + kEpsIntake);
    } else {
      update_min_intake_back_goal(kCollisionZoneIntake + kEpsIntake);
    }

    // If the intake is in the way, limit the turret until moved. Otherwise,
    // let'errip!
    if (!turret_pos_unsafe && (intake_position <= kCollisionZoneIntake)) {
      if (turret_position < min_turret_collision_goal_unwrapped) {
        update_max_turret_goal(min_turret_collision_goal_unwrapped -
                               kEpsTurret);
      } else {
        update_min_turret_goal(max_turret_collision_goal_unwrapped +
                               kEpsTurret);
      }
    }
  }
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022
