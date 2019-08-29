#ifndef Y2019_CONTROL_LOOPS_SUPERSTRUCTURE_COLLISION_AVOIDANCE_H_
#define Y2019_CONTROL_LOOPS_SUPERSTRUCTURE_COLLISION_AVOIDANCE_H_

#include <cmath>

#include "frc971/constants.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/profiled_subsystem_generated.h"
#include "y2019/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2019/control_loops/superstructure/superstructure_status_generated.h"

namespace y2019 {
namespace control_loops {
namespace superstructure {

// CollisionAvoidance computes the min and max allowable ranges for various
// subsystems to avoid collisions.  It also shoves the elevator up to let the
// intake go in and out, and to let the wrist switch sides.
class CollisionAvoidance {
 public:
  CollisionAvoidance();

  // Reports if the superstructure is collided.
  bool IsCollided(const Status *status);
  bool IsCollided(double wrist_position, double elevator_position,
                  double intake_position, bool has_piece);

  // Checks and alters goals to make sure they're safe.
  // TODO(austin): Either we will have a unit delay because this has to happen
  // after the controls, or we need to be more clever about restructuring.
  void UpdateGoal(const Status *status, const Goal *unsafe_goal);

  // Returns the goals to give to the respective control loops in
  // superstructure.
  double min_wrist_goal() const { return min_wrist_goal_; }
  double max_wrist_goal() const { return max_wrist_goal_; }
  double min_elevator_goal() const { return min_elevator_goal_; }
  double min_intake_goal() const { return min_intake_goal_; }
  double max_intake_goal() const { return max_intake_goal_; }

  void update_max_wrist_goal(double max_wrist_goal) {
    max_wrist_goal_ = ::std::min(max_wrist_goal, max_wrist_goal_);
  }
  void update_min_wrist_goal(double min_wrist_goal) {
    min_wrist_goal_ = ::std::max(min_wrist_goal, min_wrist_goal_);
  }
  void update_max_intake_goal(double max_intake_goal) {
    max_intake_goal_ = ::std::min(max_intake_goal, max_intake_goal_);
  }
  void update_min_intake_goal(double min_intake_goal) {
    min_intake_goal_ = ::std::max(min_intake_goal, min_intake_goal_);
  }
  void update_min_elevator_goal(double min_elevator_goal) {
    min_elevator_goal_ = ::std::max(min_elevator_goal, min_elevator_goal_);
  }

  // Height above which we can move the wrist freely.
  static constexpr double kElevatorClearHeight = 0.35;

  // Height above which we can move the wrist down.
  static constexpr double kElevatorClearWristDownHeight = 0.25;
  // Height the carriage needs to be above to move the intake.
  static constexpr double kElevatorClearIntakeHeight = 0.4;

  // Angle constraints for the wrist when below kElevatorClearDownHeight
  static constexpr double kWristMaxAngle = M_PI / 2.0 + 0.15;
  static constexpr double kWristMinAngle = -M_PI / 2.0 - 0.25;

  // Angles outside of which the intake is fully clear of the wrist.
  static constexpr double kIntakeOutAngle = 0.3;
  static constexpr double kIntakeInAngle = -1.1;

  // Angles within which we will crash the wrist into the elevator if the
  // elevator is below kElevatorClearHeight.
  static constexpr double kWristElevatorCollisionMinAngle = -M_PI / 4.0;
  static constexpr double kWristElevatorCollisionMaxAngle = M_PI / 4.0;
  static constexpr double kWristElevatorCollisionMaxAngleWithoutObject = M_PI / 6.0;

  // Tolerance for the elevator.
  static constexpr double kEps = 0.02;
  // Tolerance for the intake.
  static constexpr double kEpsIntake = 0.05;
  // Tolerance for the wrist.
  static constexpr double kEpsWrist = 0.05;

 private:
  void clear_min_wrist_goal() {
    min_wrist_goal_ = -::std::numeric_limits<double>::infinity();
  }
  void clear_max_wrist_goal() {
    max_wrist_goal_ = ::std::numeric_limits<double>::infinity();
  }
  void clear_min_elevator_goal() {
    min_elevator_goal_ = -::std::numeric_limits<double>::infinity();
  }
  void clear_min_intake_goal() {
    min_intake_goal_ = -::std::numeric_limits<double>::infinity();
  }
  void clear_max_intake_goal() {
    max_intake_goal_ = ::std::numeric_limits<double>::infinity();
  }

  double min_wrist_goal_;
  double max_wrist_goal_;
  double min_elevator_goal_;
  double min_intake_goal_;
  double max_intake_goal_;
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2019

#endif  // Y2019_CONTROL_LOOPS_SUPERSTRUCTURE_COLLISION_AVOIDANCE_H_
