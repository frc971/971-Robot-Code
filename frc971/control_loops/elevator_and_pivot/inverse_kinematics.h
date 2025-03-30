#pragma once

#include "absl/log/check.h"

namespace frc971::control_loops::elevator_pivot {
template <typename LocalScalar>
struct Cartesian {
  LocalScalar x;
  LocalScalar y;
};

// angle from postivie x-axis
template <typename LocalScalar>
struct PointAngle {
  Cartesian<LocalScalar> point;
  LocalScalar theta;
};

// holds the solution to the elevator pivot IK
// this system is a RP manipulator, one prismatic and one revolute
template <typename LocalScalar>
struct Solution {
  LocalScalar theta;
  LocalScalar distance;
};

// given an arm length and goal position, find elevator height and pivot
// angle(from vertical 0), first result will always have the smaller absolute
// angle
template <typename LocalScalar>
std::pair<Solution<LocalScalar>, Solution<LocalScalar>> solve_ik(
    Cartesian<LocalScalar> goal, LocalScalar arm_length) {
  CHECK(arm_length > 0) << "Arm must be positive and nonzero";
  CHECK(std::abs(goal.x) <= arm_length) << "IK is actually achievable";
  LocalScalar theta = std::asin(goal.x / arm_length);
  return {{theta, goal.y - arm_length * std::cos(theta)},
          {-std::copysign(1.0, theta) * (std::numbers::pi - std::abs(theta)),
           goal.y + arm_length * std::cos(theta)}};
}

// given a point and angle, either give the ik for that point if reachable or
// give the closest state position to that point constained to a line formed by
// an angle from that point
// This is useful for placing a game piece or picking it up when the wrist point
// must be parallel to a placing/picking up surface. This does assume that the
// game piece will take a linear trajectory which is technically not true but
// probably a good enough approximation.
// always returns the "more" useful
// solution, ie the smaller angle from vertical or the solution towards the goal
template <typename LocalScalar>
std::pair<Solution<LocalScalar>, Solution<LocalScalar>> solve_ik(
    PointAngle<LocalScalar> goal, LocalScalar arm_length) {
  CHECK(arm_length > 0) << "Arm must be positive and nonzero";
  bool is_achievable = std::abs(goal.point.x) <= arm_length;
  if (is_achievable) {
    // do normal ik
    return solve_ik(goal.point, arm_length);
  } else {
    // straight out to get maximum away from robot, while remaining on the line
    // formed by the point and angle
    // positive y solution is returned first
    LocalScalar l = (arm_length - goal.point.x) / std::cos(goal.theta);
    Solution result = {std::numbers::pi / 2.0,
                       goal.point.y + l * std::sin(-goal.theta)};
    l = (-arm_length - goal.point.x) / std::cos(goal.theta);
    Solution result2 = {-std::numbers::pi / 2.0,
                        goal.point.y + l * std::sin(-goal.theta)};
    // in the same direction if angle sign is opposite to goal x sign
    // coordinate system is if you're looking at it from the side: z is towards
    // you, y is up(elevator) and x is right
    if (result.theta * goal.point.x >= 0)
      return {result, result2};
    else
      return {result2, result};
  }
}

// returns point angle from a setpoint with respect to the robot
// wrist angle 0 is parallel to pivot and pivot zero is vertical
// if manual angle is true, wrist_angle is interpreted as the angle from the
// positive x-axis as used in the rest of the code
template <typename LocalScalar>
PointAngle<LocalScalar> from_setpoint(LocalScalar elevator_height,
                                      LocalScalar arm_angle,
                                      LocalScalar arm_length,
                                      LocalScalar wrist_angle,
                                      bool manual_angle = false) {
  return {{arm_length * std::sin(arm_angle),
           elevator_height + arm_length * std::cos(arm_angle)},
          manual_angle ? wrist_angle
                       : wrist_angle + arm_angle + std::numbers::pi / 2.0};
}

}  // namespace frc971::control_loops::elevator_pivot
