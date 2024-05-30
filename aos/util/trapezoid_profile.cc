#include "aos/util/trapezoid_profile.h"

#include <math.h>

#include <algorithm>
#include <cstdlib>
#include <ostream>

#include "glog/logging.h"

#include "aos/time/time.h"

namespace aos::util {

AsymmetricTrapezoidProfile::AsymmetricTrapezoidProfile(
    ::std::chrono::nanoseconds delta_time)
    : timestep_(delta_time) {
  output_.setZero();
}

void AsymmetricTrapezoidProfile::UpdateVals(double acceleration,
                                            double delta_time) {
  output_(0) +=
      output_(1) * delta_time + 0.5 * acceleration * delta_time * delta_time;
  output_(1) += acceleration * delta_time;
}

const Eigen::Matrix<double, 2, 1> &AsymmetricTrapezoidProfile::Update(
    double goal_position, double goal_velocity) {
  CalculateTimes(goal_position - output_(0), goal_velocity, output_);

  double next_timestep = ::aos::time::DurationInSeconds(timestep_);

  if (deceleration_reversal_time_ > next_timestep) {
    UpdateVals(deceleration_reversal_, next_timestep);
    return output_;
  }

  UpdateVals(deceleration_reversal_, deceleration_reversal_time_);
  next_timestep -= deceleration_reversal_time_;

  if (acceleration_time_ > next_timestep) {
    UpdateVals(acceleration_, next_timestep);
    return output_;
  }

  UpdateVals(acceleration_, acceleration_time_);
  next_timestep -= acceleration_time_;

  if (constant_time_ > next_timestep) {
    UpdateVals(0, next_timestep);
    return output_;
  }

  UpdateVals(0, constant_time_);
  next_timestep -= constant_time_;
  if (deceleration_time_ > next_timestep) {
    UpdateVals(deceleration_, next_timestep);
  } else {
    UpdateVals(deceleration_, deceleration_time_);
    next_timestep -= deceleration_time_;
    UpdateVals(0, next_timestep);

    if (next_timestep >= 0 && goal_velocity == 0) {
      output_(0) = goal_position;
      output_(1) = goal_velocity;
    }
  }

  return output_;
}

void AsymmetricTrapezoidProfile::CalculateTimes(
    double distance_to_target, double goal_velocity,
    Eigen::Matrix<double, 2, 1> current) {
  if (distance_to_target == 0) {
    // We're there. Stop everything.
    // TODO(aschuh): Deal with velocity not right.
    acceleration_time_ = 0;
    acceleration_ = 0;
    constant_time_ = 0;
    deceleration_time_ = 0;
    deceleration_ = 0;
    deceleration_reversal_time_ = 0;
    deceleration_reversal_ = 0;
    return;
  } else if (distance_to_target < 0) {
    // Recurse with everything inverted.
    current(1) *= -1;
    CalculateTimes(-distance_to_target, -goal_velocity, current);
    acceleration_ *= -1;
    deceleration_ *= -1;
    deceleration_reversal_ *= -1;
    return;
  }

  constant_time_ = 0;
  acceleration_ = maximum_acceleration_;

  // Calculate the fastest speed we could get going to by the distance to
  // target.  We will have normalized everything out to be a positive distance
  // by now so we never have to deal with going "backwards".
  double maximum_acceleration_velocity =
      distance_to_target * 2 * std::abs(acceleration_) +
      current(1) * current(1);
  CHECK_GE(maximum_acceleration_velocity, 0);
  maximum_acceleration_velocity = sqrt(maximum_acceleration_velocity);

  // If we could get going faster than the target, we will need to decelerate
  // after accelerating.
  if (maximum_acceleration_velocity > goal_velocity) {
    deceleration_ = -maximum_deceleration_;
  } else {
    // We couldn't get up to speed by the destination.  Set our decel to
    // accelerate to keep accelerating to get up to speed.
    //
    // Note: goal_velocity != 0 isn't well tested, use at your own risk.
    LOG(FATAL) << "Untested";
    deceleration_ = maximum_acceleration_;
  }

  // If we are going away from the goal, we will need to change directions.
  if (current(1) < 0) {
    deceleration_reversal_time_ = current(1) / deceleration_;
    deceleration_reversal_ = -deceleration_;
  } else if ((goal_velocity - current(1)) * (goal_velocity + current(1)) <
             2.0 * deceleration_ * distance_to_target) {
    // Then, can we stop in time if we get after it?  If so, we don't need to
    // decel first before running the profile.
    deceleration_reversal_time_ = -current(1) / deceleration_;
    deceleration_reversal_ = deceleration_;
  } else {
    // Otherwise, we are all good and don't need to handle reversing.
    deceleration_reversal_time_ = 0.0;
    deceleration_reversal_ = 0.0;
  }

  current(0) += current(1) * deceleration_reversal_time_ +
                0.5 * deceleration_reversal_ * deceleration_reversal_time_ *
                    deceleration_reversal_time_;
  current(1) += deceleration_reversal_ * deceleration_reversal_time_;
  // OK, now we've compensated for slowing down.

  // We now know the top velocity we can get to.
  const double top_velocity = sqrt(
      (distance_to_target + (current(1) * current(1)) / (2.0 * acceleration_) +
       (goal_velocity * goal_velocity) / (2.0 * deceleration_)) /
      (-1.0 / (2.0 * deceleration_) + 1.0 / (2.0 * acceleration_)));

  // If it can go too fast, we now know how long we get to accelerate for and
  // how long to go at constant velocity.
  if (top_velocity > maximum_velocity_) {
    acceleration_time_ =
        (maximum_velocity_ - current(1)) / maximum_acceleration_;
    constant_time_ = ((-0.5 * maximum_acceleration_ * acceleration_time_ *
                           acceleration_time_ -
                       current(1) * acceleration_time_) +
                      distance_to_target +
                      (goal_velocity * goal_velocity -
                       maximum_velocity_ * maximum_velocity_) /
                          (2.0 * maximum_deceleration_)) /
                     maximum_velocity_;
  } else {
    acceleration_time_ = (top_velocity - current(1)) / acceleration_;
  }

  CHECK_GT(top_velocity, -maximum_velocity_);

  if (current(1) > maximum_velocity_) {
    constant_time_ = 0;
    acceleration_time_ = 0;
  }

  deceleration_time_ =
      (goal_velocity - ::std::min(top_velocity, maximum_velocity_)) /
      deceleration_;
  if (acceleration_time_ <= 0) acceleration_time_ = 0;
  if (constant_time_ <= 0) constant_time_ = 0;
  if (deceleration_time_ <= 0) deceleration_time_ = 0;
}

}  // namespace aos::util
