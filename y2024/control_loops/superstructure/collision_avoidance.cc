#include "y2024/control_loops/superstructure/collision_avoidance.h"

#include <cmath>

#include "absl/functional/bind_front.h"
#include "glog/logging.h"

namespace y2024::control_loops::superstructure {

CollisionAvoidance::CollisionAvoidance() {
  clear_min_intake_pivot_goal();
  clear_max_intake_pivot_goal();
  clear_min_turret_goal();
  clear_max_turret_goal();
}

bool CollisionAvoidance::IsCollided(const CollisionAvoidance::Status &status) {
  // Checks if intake front is collided.
  if (TurretCollided(status.intake_pivot_position, status.turret_position,
                     kMinCollisionZoneTurret, kMaxCollisionZoneTurret)) {
    return true;
  }

  return false;
}

bool AngleInRange(double theta, double theta_min, double theta_max) {
  return (
      (theta >= theta_min && theta <= theta_max) ||
      (theta_min > theta_max && (theta >= theta_min || theta <= theta_max)));
}

bool CollisionAvoidance::TurretCollided(double intake_position,
                                        double turret_position,
                                        double min_turret_collision_position,
                                        double max_turret_collision_position) {
  // Checks if turret is in the collision area.
  if (AngleInRange(turret_position, min_turret_collision_position,
                   max_turret_collision_position)) {
    // Returns true if the intake is raised.
    if (intake_position > kCollisionZoneIntake) {
      return true;
    }
  } else {
    return false;
  }
  return false;
}

void CollisionAvoidance::UpdateGoal(const CollisionAvoidance::Status &status,
                                    const double &turret_goal_position) {
  // Start with our constraints being wide open.
  clear_max_turret_goal();
  clear_min_turret_goal();
  clear_max_intake_pivot_goal();
  clear_min_intake_pivot_goal();

  const double intake_pivot_position = status.intake_pivot_position;
  const double turret_position = status.turret_position;

  const double turret_goal = turret_goal_position;

  // Calculating the avoidance with the intake

  CalculateAvoidance(true, intake_pivot_position, turret_position, turret_goal,
                     kMinCollisionZoneTurret, kMaxCollisionZoneTurret);
}

void CollisionAvoidance::CalculateAvoidance(bool intake_pivot,
                                            double intake_position,
                                            double turret_position,
                                            double turret_goal,
                                            double min_turret_collision_goal,
                                            double max_turret_collision_goal) {
  // If the turret goal is in a collison zone or moving through one, limit
  // intake.
  const bool turret_pos_unsafe = AngleInRange(
      turret_position, min_turret_collision_goal, max_turret_collision_goal);

  const bool turret_moving_forward = (turret_goal > turret_position);

  // Check if the closest angles are going to be passed
  const bool turret_moving_past_intake =
      ((turret_moving_forward &&
        (turret_position <= max_turret_collision_goal &&
         turret_goal >= min_turret_collision_goal)) ||
       (!turret_moving_forward &&
        (turret_position >= min_turret_collision_goal &&
         turret_goal <= max_turret_collision_goal)));

  if (turret_pos_unsafe || turret_moving_past_intake) {
    // If the turret is unsafe, limit the intake
    if (intake_pivot) {
      update_max_intake_pivot_goal(kCollisionZoneIntake - kEpsIntake);
    }

    // If the intake is in the way, limit the turret until moved. Otherwise,
    // let'errip!
    if (!turret_pos_unsafe && (intake_position > kCollisionZoneIntake)) {
      if (turret_position <
          (min_turret_collision_goal + max_turret_collision_goal / 2)) {
        update_max_turret_goal(min_turret_collision_goal - kEpsTurret);
      } else {
        update_min_turret_goal(max_turret_collision_goal + kEpsTurret);
      }
    }
  }
}

}  // namespace y2024::control_loops::superstructure
