#ifndef Y2024_CONTROL_LOOPS_SUPERSTRUCTURE_COLLISION_AVOIDENCE_H_
#define Y2024_CONTROL_LOOPS_SUPERSTRUCTURE_COLLISION_AVOIDENCE_H_

#include <cmath>

#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/profiled_subsystem_generated.h"

namespace y2024::control_loops::superstructure {

// Checks if theta is between theta_min and theta_max. Expects all angles to be
// wrapped from 0 to 2pi
bool AngleInRange(double theta, double theta_min, double theta_max);

// 1. Prevent the turret from moving if the intake is up
// and prevent the back of the turret from colliding with the intake when it's
// up.
// 2. If the intake is up, drop it so it is not in the way
// 3. Move the turret to the desired position.
// 4. When the turret moves away, if the intake is down, move it back up.
class CollisionAvoidance {
 public:
  struct Status {
    double intake_pivot_position;
    double turret_position;
    double extend_position;

    bool operator==(const Status &s) const {
      return (intake_pivot_position == s.intake_pivot_position &&
              turret_position == s.turret_position &&
              extend_position == s.extend_position);
    }
    bool operator!=(const Status &s) const { return !(*this == s); }
  };

  // For the turret, 0 rad is pointing straight forwards
  static constexpr double kMinCollisionZoneTurret = 0.15;
  static constexpr double kMaxCollisionZoneTurret = 1.15;

  static constexpr double kSafeTurretExtendedPosition = 0.0;

  // Maximum position of the intake to avoid collisions
  static constexpr double kCollisionZoneIntake = 1.6;

  static constexpr double kMinCollisionZoneExtend = 0.03;

  // Tolerances for the subsystems
  static constexpr double kEpsTurret = 0.05;
  static constexpr double kEpsIntake = 0.05;
  static constexpr double kEpsExtend = 0.005;

  CollisionAvoidance();

  // Reports if the superstructure is collided.
  bool IsCollided(const Status &status);
  // Checks if there is a collision with the intake.
  bool TurretCollided(double intake_position, double turret_position,
                      double extend_position);
  // Checks if there is a collision with the extend.
  bool ExtendCollided(double intake_position, double turret_position,
                      double extend_position);
  // Checks and alters goals to make sure they're safe.
  void UpdateGoal(const CollisionAvoidance::Status &status,
                  double turret_goal_position, double extend_goal_position);
  // Limits if goal is in collision spots.
  void CalculateAvoidance(double intake_position, double turret_position,
                          double extend_position, double turret_goal,
                          double extend_goal);

  // Returns the goals to give to the respective control loops in
  // superstructure.
  double min_extend_goal() const { return min_extend_goal_; }
  double max_extend_goal() const { return max_extend_goal_; }
  double min_turret_goal() const { return min_turret_goal_; }
  double max_turret_goal() const { return max_turret_goal_; }
  double min_intake_pivot_goal() const { return min_intake_pivot_goal_; }
  double max_intake_pivot_goal() const { return max_intake_pivot_goal_; }

  void update_max_turret_goal(double max_turret_goal) {
    max_turret_goal_ = ::std::min(max_turret_goal, max_turret_goal_);
  }
  void update_min_turret_goal(double min_turret_goal) {
    min_turret_goal_ = ::std::max(min_turret_goal, min_turret_goal_);
  }
  void update_max_intake_pivot_goal(double max_intake_pivot_goal) {
    max_intake_pivot_goal_ =
        ::std::min(max_intake_pivot_goal, max_intake_pivot_goal_);
  }
  void update_min_intake_pivot_goal(double min_intake_pivot_goal) {
    min_intake_pivot_goal_ =
        ::std::max(min_intake_pivot_goal, min_intake_pivot_goal_);
  }
  void update_min_extend_goal(double min_extend_goal) {
    min_extend_goal_ = ::std::max(min_extend_goal, min_extend_goal_);
  }
  void update_max_extend_goal(double max_extend_goal) {
    max_extend_goal_ = ::std::min(max_extend_goal, max_extend_goal_);
  }

 private:
  void clear_min_intake_pivot_goal() {
    min_intake_pivot_goal_ = -::std::numeric_limits<double>::infinity();
  }
  void clear_max_intake_pivot_goal() {
    max_intake_pivot_goal_ = ::std::numeric_limits<double>::infinity();
  }
  void clear_min_turret_goal() {
    min_turret_goal_ = -::std::numeric_limits<double>::infinity();
  }
  void clear_max_turret_goal() {
    max_turret_goal_ = ::std::numeric_limits<double>::infinity();
  }
  void clear_min_extend_goal() {
    min_extend_goal_ = -::std::numeric_limits<double>::infinity();
  }
  void clear_max_extend_goal() {
    max_extend_goal_ = ::std::numeric_limits<double>::infinity();
  }

  double min_intake_pivot_goal_;
  double max_intake_pivot_goal_;
  double min_turret_goal_;
  double max_turret_goal_;
  double min_extend_goal_;
  double max_extend_goal_;
};

}  // namespace y2024::control_loops::superstructure

#endif
