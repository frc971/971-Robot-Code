#ifndef AOS_UTIL_TRAPEZOID_PROFILE_H_
#define AOS_UTIL_TRAPEZOID_PROFILE_H_

#include "Eigen/Dense"

#include "aos/macros.h"
#include "aos/time/time.h"

namespace aos::util {

// Calculates a trapezoidal motion profile (like for a control loop's goals).
// Supports having the destination position, acceleration, and velocity caps
// changed in the middle, and for having different accelerations and
// decelerations.
//
// The only units assumption that this class makes is that the unit of time is
// seconds.
class AsymmetricTrapezoidProfile {
 public:
  // Constructs a profile.  delta_time is the timestep to assume when solving.
  AsymmetricTrapezoidProfile(::std::chrono::nanoseconds delta_time);

  // Updates the state to provide the next position and velocity to go to to
  // follow the profile.
  const Eigen::Matrix<double, 2, 1> &Update(double goal_position,
                                            double goal_velocity);

  // Updates the internal position.  Useful for handling windup when a loop
  // saturates or gets disabled.
  void MoveCurrentState(const Eigen::Matrix<double, 2, 1> &current) {
    output_ = current;
  }

  // Adjusts the internal position by the provided position delta.
  void MoveGoal(double dx) { output_(0, 0) += dx; }

  // Sets the internal position to the provided position.
  void SetGoal(double x) { output_(0, 0) = x; }

  void set_maximum_acceleration(double maximum_acceleration) {
    maximum_acceleration_ = maximum_acceleration;
  }
  void set_maximum_deceleration(double maximum_deceleration) {
    maximum_deceleration_ = maximum_deceleration;
  }

  void set_maximum_velocity(double maximum_velocity) {
    maximum_velocity_ = maximum_velocity;
  }

 private:
  // Updates output_ to match the basic kinematics, given that we are going to
  // accelerate by acceleration over delta_time.
  void UpdateVals(double acceleration, double delta_time);
  // Calculates how long to go for each segment.
  void CalculateTimes(double distance_to_target, double goal_velocity,
                      Eigen::Matrix<double, 2, 1> current);
  // output_ is where it should go at time_.
  Eigen::Matrix<double, 2, 1> output_;

  // Time and acceleration to slow down if we need to reverse directions.
  double deceleration_reversal_time_;
  double deceleration_reversal_;

  // Time and acceleration to speed up with.
  double acceleration_time_;
  double acceleration_;
  // Time to go at max speed at.
  double constant_time_;
  // Time and acceleration to slow down with.
  double deceleration_time_;
  double deceleration_;

  double maximum_acceleration_ = 0;
  double maximum_deceleration_ = 0;
  double maximum_velocity_ = 0;

  // How long between calls to Update.
  ::std::chrono::nanoseconds timestep_;

  DISALLOW_COPY_AND_ASSIGN(AsymmetricTrapezoidProfile);
};

// Class to implement a AsymmetricTrapezoidProfile where both acceleration and
// deceleration match.
class TrapezoidProfile {
 public:
  TrapezoidProfile(::std::chrono::nanoseconds delta_time)
      : asymmetric_trapezoid_profile_(delta_time) {}

  // Updates the state to provide the next position and velocity to go to to
  // follow the profile.
  const Eigen::Matrix<double, 2, 1> &Update(double goal_position,
                                            double goal_velocity) {
    return asymmetric_trapezoid_profile_.Update(goal_position, goal_velocity);
  }

  // Updates the internal position.  Useful for handling windup when a loop
  // saturates or gets disabled.
  void MoveCurrentState(const Eigen::Matrix<double, 2, 1> &current) {
    asymmetric_trapezoid_profile_.MoveCurrentState(current);
  }

  // Adjusts the internal position by the provided position delta.
  void MoveGoal(double dx) { asymmetric_trapezoid_profile_.MoveGoal(dx); }

  // Sets the internal position to the provided position.
  void SetGoal(double x) { asymmetric_trapezoid_profile_.SetGoal(x); }

  void set_maximum_acceleration(double maximum_acceleration) {
    asymmetric_trapezoid_profile_.set_maximum_acceleration(
        maximum_acceleration);
    asymmetric_trapezoid_profile_.set_maximum_deceleration(
        maximum_acceleration);
  }
  void set_maximum_velocity(double maximum_velocity) {
    asymmetric_trapezoid_profile_.set_maximum_velocity(maximum_velocity);
  }

 private:
  AsymmetricTrapezoidProfile asymmetric_trapezoid_profile_;
};

}  // namespace aos::util

#endif  // AOS_UTIL_TRAPEZOID_PROFILE_H_
