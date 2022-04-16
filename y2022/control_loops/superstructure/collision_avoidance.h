#ifndef Y2022_CONTROL_LOOPS_SUPERSTRUCTURE_COLLISION_AVOIDENCE_H_
#define Y2022_CONTROL_LOOPS_SUPERSTRUCTURE_COLLISION_AVOIDENCE_H_

#include <cmath>

#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/profiled_subsystem_generated.h"
#include "y2022/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2022/control_loops/superstructure/superstructure_status_generated.h"

namespace y2022 {
namespace control_loops {
namespace superstructure {

// Returns the wrapped angle as well as number of wraps (positive or negative).
// The returned angle will be inside [0.0, 2 * M_PI).
std::pair<double, int> WrapTurretAngle(double turret_angle);

// Returns the absolute angle given the wrapped angle and number of wraps.
double UnwrapTurretAngle(double wrapped, int wraps);

// Checks if theta is between theta_min and theta_max. Expects all angles to be
// wrapped from 0 to 2pi
bool AngleInRange(double theta, double theta_min, double theta_max);

// 1. Prevent the turret from moving if the intake is up
// and prevent the back of the turret (where the catapult is)
// from colliding with the intake when it's up.
// 2. If the intake is up, drop it so it is not in the way
// 3. Move the turret to the desired position.
// 4. When the turret moves away, if the intake is down, move it back up.
class CollisionAvoidance {
 public:
  struct Status {
    double intake_front_position;
    double intake_back_position;
    double turret_position;
    bool shooting;

    bool operator==(const Status &s) const {
      return (intake_front_position == s.intake_front_position &&
              intake_back_position == s.intake_back_position &&
              turret_position == s.turret_position && shooting == s.shooting);
    }
    bool operator!=(const Status &s) const { return !(*this == s); }
  };

  // TODO(henry): put actual constants here.

  // Reference angles between which the turret will be careful
  static constexpr double kCollisionZoneTurret = M_PI * 7.0 / 18.0;

  // For the turret, 0 rad is pointing straight forwards
  static constexpr double kMinCollisionZoneFrontTurret =
      M_PI - kCollisionZoneTurret;
  static constexpr double kMaxCollisionZoneFrontTurret =
      M_PI + kCollisionZoneTurret;

  static constexpr double kMinCollisionZoneBackTurret =
      (2.0 * M_PI) - kCollisionZoneTurret;
  static constexpr double kMaxCollisionZoneBackTurret = kCollisionZoneTurret;

  // Maximum position of the intake to avoid collisions
  static constexpr double kCollisionZoneIntake = 1.30;

  // Tolerances for the subsystems
  static constexpr double kEpsTurret = 0.05;
  static constexpr double kEpsIntake = 0.05;

  CollisionAvoidance();

  // Reports if the superstructure is collided.
  bool IsCollided(const Status &status);
  // Checks if there is a collision on either intake.
  bool TurretCollided(double intake_position, double turret_position,
                      double min_turret_collision_position,
                      double max_turret_collision_position);
  // Checks and alters goals to make sure they're safe.
  void UpdateGoal(
      const Status &status,
      const frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal
          *unsafe_turret_goal);
  // Limits if goal is in collision spots.
  void CalculateAvoidance(bool intake_front, bool catapult,
                          double intake_position, double turret_position,
                          double turret_goal, double min_turret_collision_goal,
                          double max_turret_collision_goal);

  // Returns the goals to give to the respective control loops in
  // superstructure.
  double min_turret_goal() const { return min_turret_goal_; }
  double max_turret_goal() const { return max_turret_goal_; }
  double min_intake_front_goal() const { return min_intake_front_goal_; }
  double max_intake_front_goal() const { return max_intake_front_goal_; }
  double min_intake_back_goal() const { return min_intake_back_goal_; }
  double max_intake_back_goal() const { return max_intake_back_goal_; }

  void update_max_turret_goal(double max_turret_goal) {
    max_turret_goal_ = ::std::min(max_turret_goal, max_turret_goal_);
  }
  void update_min_turret_goal(double min_turret_goal) {
    min_turret_goal_ = ::std::max(min_turret_goal, min_turret_goal_);
  }
  void update_max_intake_front_goal(double max_intake_front_goal) {
    max_intake_front_goal_ =
        ::std::min(max_intake_front_goal, max_intake_front_goal_);
  }
  void update_min_intake_front_goal(double min_intake_front_goal) {
    min_intake_front_goal_ =
        ::std::max(min_intake_front_goal, min_intake_front_goal_);
  }
  void update_max_intake_back_goal(double max_intake_back_goal) {
    max_intake_back_goal_ =
        ::std::min(max_intake_back_goal, max_intake_back_goal_);
  }
  void update_min_intake_back_goal(double min_intake_back_goal) {
    min_intake_back_goal_ =
        ::std::max(min_intake_back_goal, min_intake_back_goal_);
  }

 private:
  void clear_min_intake_front_goal() {
    min_intake_front_goal_ = -::std::numeric_limits<double>::infinity();
  }
  void clear_max_intake_front_goal() {
    max_intake_front_goal_ = ::std::numeric_limits<double>::infinity();
  }
  void clear_min_intake_back_goal() {
    min_intake_back_goal_ = -::std::numeric_limits<double>::infinity();
  }
  void clear_max_intake_back_goal() {
    max_intake_back_goal_ = ::std::numeric_limits<double>::infinity();
  }
  void clear_min_turret_goal() {
    min_turret_goal_ = -::std::numeric_limits<double>::infinity();
  }
  void clear_max_turret_goal() {
    max_turret_goal_ = ::std::numeric_limits<double>::infinity();
  }

  double min_intake_front_goal_;
  double max_intake_front_goal_;
  double min_intake_back_goal_;
  double max_intake_back_goal_;
  double min_turret_goal_;
  double max_turret_goal_;
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022

#endif
