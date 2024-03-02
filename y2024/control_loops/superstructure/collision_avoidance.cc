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
  clear_min_extend_goal();
  clear_max_extend_goal();
}

bool CollisionAvoidance::IsCollided(const CollisionAvoidance::Status &status) {
  // Checks if intake front is collided.
  if (TurretCollided(status.intake_pivot_position, status.turret_position,
                     status.extend_position)) {
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
                                        double extend_position) {
  // Checks if turret is in the collision area.
  if (AngleInRange(turret_position, kMinCollisionZoneTurret,
                   kMaxCollisionZoneTurret)) {
    // Returns true if the intake is raised.
    if (intake_position > kCollisionZoneIntake) {
      return true;
    }
  }
  return ExtendCollided(intake_position, turret_position, extend_position);
}

bool CollisionAvoidance::ExtendCollided(double /*intake_position*/,
                                        double turret_position,
                                        double extend_position) {
  // Checks if turret is in the collision area.
  if (!AngleInRange(turret_position, kSafeTurretExtendedPosition - kEpsTurret,
                    kSafeTurretExtendedPosition + kEpsTurret)) {
    // Returns true if the extend is raised.
    if (extend_position > kMinCollisionZoneExtend) {
      return true;
    }
  }

  return false;
}

void CollisionAvoidance::UpdateGoal(const CollisionAvoidance::Status &status,
                                    const double turret_goal_position,
                                    const double extend_goal_position) {
  // Start with our constraints being wide open.
  clear_min_extend_goal();
  clear_max_extend_goal();
  clear_max_turret_goal();
  clear_min_turret_goal();
  clear_max_intake_pivot_goal();
  clear_min_intake_pivot_goal();

  const double intake_pivot_position = status.intake_pivot_position;
  const double turret_position = status.turret_position;
  const double extend_position = status.extend_position;

  const double turret_goal = turret_goal_position;
  const double extend_goal = extend_goal_position;

  // Calculating the avoidance with the intake

  CalculateAvoidance(intake_pivot_position, turret_position, extend_position,
                     turret_goal, extend_goal);
}

void CollisionAvoidance::CalculateAvoidance(double intake_position,
                                            double turret_position,
                                            double extend_position,
                                            double turret_goal,
                                            double extend_goal) {
  // If the turret goal is in a collison zone or moving through one, limit
  // intake.
  const bool turret_intake_pos_unsafe = AngleInRange(
      turret_position, kMinCollisionZoneTurret, kMaxCollisionZoneTurret);
  const bool turret_extend_pos_unsafe =
      turret_position > kEpsTurret + kSafeTurretExtendedPosition ||
      turret_position < -kEpsTurret + kSafeTurretExtendedPosition;

  const bool extend_goal_unsafe =
      extend_goal > kMinCollisionZoneExtend - kEpsExtend;
  const bool extend_position_unsafe =
      extend_position > kMinCollisionZoneExtend - kEpsExtend;

  // OK, we are trying to move the extend, and need the turret to be at 0.
  // Pretend that's the goal.
  if (extend_goal_unsafe || extend_position_unsafe) {
    turret_goal = kSafeTurretExtendedPosition;
  }

  const bool turret_moving_forward = (turret_goal > turret_position);

  // Check if the closest angles are going to be passed
  const bool turret_moving_past_intake =
      ((turret_moving_forward && (turret_position <= kMaxCollisionZoneTurret &&
                                  turret_goal >= kMinCollisionZoneTurret)) ||
       (!turret_moving_forward && (turret_position >= kMinCollisionZoneTurret &&
                                   turret_goal <= kMaxCollisionZoneTurret)));

  if (turret_intake_pos_unsafe || turret_moving_past_intake) {
    // If the turret is unsafe, limit the intake
    update_max_intake_pivot_goal(kCollisionZoneIntake - kEpsIntake);

    // If the intake is in the way, limit the turret until moved. Otherwise,
    // let'errip!
    if (!turret_intake_pos_unsafe && (intake_position > kCollisionZoneIntake)) {
      if (turret_position <
          (kMinCollisionZoneTurret + kMaxCollisionZoneTurret) / 2.) {
        update_max_turret_goal(kMinCollisionZoneTurret - kEpsTurret);
      } else {
        update_min_turret_goal(kMaxCollisionZoneTurret + kEpsTurret);
      }
    }
  }

  // OK, the logic is pretty simple.  The turret needs to be at
  // kSafeTurretExtendedPosition any time extend is > kMinCollisionZoneExtend.
  //
  // Extend can't go up if the turret isn't near 0.
  if (turret_extend_pos_unsafe) {
    update_max_extend_goal(kMinCollisionZoneExtend - kEpsExtend);
  }

  // Turret is bound to the safe position if extend wants to be, or is unsafe.
  if (extend_goal_unsafe || extend_position_unsafe) {
    // If the turret isn't allowed to go to 0, don't drive it there.
    if (min_turret_goal() < kSafeTurretExtendedPosition &&
        max_turret_goal() > kSafeTurretExtendedPosition) {
      update_min_turret_goal(kSafeTurretExtendedPosition);
      update_max_turret_goal(kSafeTurretExtendedPosition);
    }
  }
}

}  // namespace y2024::control_loops::superstructure
