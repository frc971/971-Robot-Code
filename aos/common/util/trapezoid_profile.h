#ifndef AOS_COMMON_UTIL_TRAPEZOID_PROFILE_H_
#define AOS_COMMON_UTIL_TRAPEZOID_PROFILE_H_

#include "Eigen/Dense"

#include "aos/common/macros.h"
#include "aos/common/time.h"

namespace aos {
namespace util {

// Calculates a trapezoidal motion profile (like for a control loop's goals).
// Supports having the end speed and position changed in the middle.
//
// The only units assumption that this class makes is that the unit of time is
// seconds.
class TrapezoidProfile {
 public:
  // delta_time is how long between each call to Update.
  TrapezoidProfile(const time::Time &delta_time);

  // Updates the state.
  const Eigen::Matrix<double, 2, 1> &Update(double goal_position,
                                            double goal_velocity);
  // Useful for preventing windup etc.
  void MoveCurrentState(const Eigen::Matrix<double, 2, 1> &current) {
    output_ = current;
  }

  // Useful for preventing windup etc.
  void MoveGoal(double dx) { output_(0, 0) += dx; }

  void SetGoal(double x) { output_(0, 0) = x; }

  void set_maximum_acceleration(double maximum_acceleration) {
    maximum_acceleration_ = maximum_acceleration;
  }
  void set_maximum_velocity(double maximum_velocity) {
    maximum_velocity_ = maximum_velocity;
  }

 private:
  // Basic kinematics to update output_, given that we are going to accelerate
  // by acceleration over delta_time.
  void UpdateVals(double acceleration, double delta_time);
  // Calculates how long to go for each segment.
  void CalculateTimes(double distance_to_target, double goal_velocity);
  // output_ is where it should go at time_.
  Eigen::Matrix<double, 2, 1> output_;

  double acceleration_time_;
  double acceleration_;
  double constant_time_;
  double deceleration_time_;
  double deceleration_;

  double maximum_acceleration_;
  double maximum_velocity_;

  // How long between calls to Update.
  const time::Time timestep_;

  DISALLOW_COPY_AND_ASSIGN(TrapezoidProfile);
};

}  // namespace util
}  // namespace aos

#endif  // AOS_COMMON_UTIL_TRAPEZOID_PROFILE_H_
