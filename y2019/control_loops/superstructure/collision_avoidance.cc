#include "y2019/control_loops/superstructure/collision_avoidance.h"

#include <cmath>

#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/profiled_subsystem_generated.h"
#include "y2019/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2019/control_loops/superstructure/superstructure_status_generated.h"

namespace y2019 {
namespace control_loops {
namespace superstructure {

constexpr double CollisionAvoidance::kElevatorClearHeight;
constexpr double CollisionAvoidance::kElevatorClearWristDownHeight;
constexpr double CollisionAvoidance::kElevatorClearIntakeHeight;
constexpr double CollisionAvoidance::kWristMaxAngle;
constexpr double CollisionAvoidance::kWristMinAngle;
constexpr double CollisionAvoidance::kIntakeOutAngle;
constexpr double CollisionAvoidance::kIntakeInAngle;
constexpr double CollisionAvoidance::kWristElevatorCollisionMinAngle;
constexpr double CollisionAvoidance::kWristElevatorCollisionMaxAngle;
constexpr double
    CollisionAvoidance::kWristElevatorCollisionMaxAngleWithoutObject;
constexpr double CollisionAvoidance::kEps;
constexpr double CollisionAvoidance::kEpsIntake;
constexpr double CollisionAvoidance::kEpsWrist;

CollisionAvoidance::CollisionAvoidance() {
  clear_min_wrist_goal();
  clear_max_wrist_goal();
  clear_min_elevator_goal();
  clear_min_intake_goal();
  clear_max_intake_goal();
}

bool CollisionAvoidance::IsCollided(const Status *status) {
  return IsCollided(status->wrist()->position(), status->elevator()->position(),
                    status->intake()->position(), status->has_piece());
}

bool CollisionAvoidance::IsCollided(const double wrist_position,
                                    const double elevator_position,
                                    const double intake_position,
                                    const bool has_piece) {
  const double wrist_elevator_collision_max_angle =
      has_piece ? kWristElevatorCollisionMaxAngle
                : kWristElevatorCollisionMaxAngleWithoutObject;

  // Elevator is down, so the wrist can't be close to vertical.
  if (elevator_position < kElevatorClearHeight) {
    if (wrist_position < wrist_elevator_collision_max_angle &&
        wrist_position > kWristElevatorCollisionMinAngle) {
      return true;
    }
  }

  // Elevator is down so wrist can't go below horizontal in either direction.
  if (elevator_position < kElevatorClearWristDownHeight) {
    if (wrist_position > kWristMaxAngle) {
      return true;
    }
    if (wrist_position < kWristMinAngle) {
      return true;
    }
  }

  // Elevator is down so the intake has to be at either extreme.
  if (elevator_position < kElevatorClearIntakeHeight &&
      wrist_position > kWristMaxAngle) {
    if (intake_position < kIntakeOutAngle && intake_position > kIntakeInAngle) {
      return true;
    }
  }

  // Nothing is hitting, we must be good.
  return false;
}

void CollisionAvoidance::UpdateGoal(const Status *status,
                                    const Goal *unsafe_goal) {
  const double wrist_position = status->wrist()->position();
  const double elevator_position = status->elevator()->position();
  const double intake_position = status->intake()->position();
  const bool has_piece = status->has_piece();

  // Start with our constraints being wide open.
  clear_max_wrist_goal();
  clear_min_wrist_goal();
  clear_max_intake_goal();
  clear_min_intake_goal();

  const double wrist_elevator_collision_max_angle =
      has_piece ? kWristElevatorCollisionMaxAngle
                : kWristElevatorCollisionMaxAngleWithoutObject;

  // If the elevator is low enough, we also can't transition the wrist.
  if (elevator_position < kElevatorClearHeight) {
    // Figure out which side the wrist is on and stay there.
    if (wrist_position < (2.0 * kWristElevatorCollisionMinAngle +
                          kWristElevatorCollisionMaxAngle) /
                             3.0) {
      update_max_wrist_goal(kWristElevatorCollisionMinAngle - kEpsWrist);
    } else {
      update_min_wrist_goal(wrist_elevator_collision_max_angle + kEpsWrist);
    }
  }

  // If the elevator is too low, the wrist needs to be above the clearance
  // angles to avoid crashing the frame.
  if (elevator_position < kElevatorClearWristDownHeight) {
    update_min_wrist_goal(kWristMinAngle + kEpsWrist);
    update_max_wrist_goal(kWristMaxAngle - kEpsWrist);
  }

  constexpr double kIntakeMiddleAngle =
      (kIntakeOutAngle + kIntakeInAngle) / 2.0;

  // If the elevator is too low, the intake can't transition from in to out or
  // back.
  if (elevator_position < kElevatorClearIntakeHeight &&
      wrist_position > wrist_elevator_collision_max_angle) {
    // Figure out if the intake is in our out and keep it there.
    if (intake_position < kIntakeMiddleAngle) {
      update_max_intake_goal(kIntakeInAngle - kEpsIntake);
    } else {
      update_min_intake_goal(kIntakeOutAngle + kEpsIntake);
    }
  }

  // Start with an unconstrained elevator.
  clear_min_elevator_goal();

  // If the intake is within the collision range, don't let the elevator down.
  if (intake_position > kIntakeInAngle && intake_position < kIntakeOutAngle &&
      wrist_position > wrist_elevator_collision_max_angle) {
    update_min_elevator_goal(kElevatorClearIntakeHeight + kEps);
  }

  // If the intake is in the collision range and the elevator is down, don't let
  // the wrist go far down.
  if (intake_position > kIntakeInAngle && intake_position < kIntakeOutAngle &&
      elevator_position < kElevatorClearIntakeHeight) {
    update_max_wrist_goal(kWristMaxAngle - kEpsWrist);
  }

  // If the wrist is within the elevator collision range, don't let the elevator
  // go down.
  if (wrist_position > kWristElevatorCollisionMinAngle &&
      wrist_position < wrist_elevator_collision_max_angle) {
    update_min_elevator_goal(kElevatorClearHeight + kEps);
  }

  // If the wrist is far enough down that we are going to hit the frame, don't
  // let the elevator go too far down.
  if (wrist_position > kWristMaxAngle || wrist_position < kWristMinAngle) {
    update_min_elevator_goal(kElevatorClearWristDownHeight + kEps);
  }

  if (unsafe_goal) {
    const double wrist_goal = unsafe_goal->wrist()->unsafe_goal();
    const double intake_goal = unsafe_goal->intake()->unsafe_goal();

    // Compute if we need to move the intake.
    const bool intake_needs_to_move = (intake_position < kIntakeMiddleAngle) ^
                                      (intake_goal < kIntakeMiddleAngle);

    // Compute if we need to move the wrist across 0.
    const bool wrist_needs_to_move =
        (wrist_position < 0.0) ^ (wrist_goal < 0.0);

    // If we need to move the intake, we've got to shove the elevator up.  The
    // intake is already constrained so it can't hit anything until it's clear.
    if (intake_needs_to_move &&
        wrist_position > wrist_elevator_collision_max_angle) {
      update_min_elevator_goal(kElevatorClearIntakeHeight + kEps);
    }
    // If we need to move the wrist, we've got to shove the elevator up too. The
    // wrist is already constrained so it can't hit anything until it's clear.
    // If both the intake and wrist need to move, figure out which one will
    // require the higher motion and move that.
    if (wrist_needs_to_move) {
      update_min_elevator_goal(kElevatorClearHeight + kEps);
    }

    // TODO(austin): We won't shove the elevator up if the wrist is asked to go
    // down below horizontal.  I think that's fine.
  }
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2019
