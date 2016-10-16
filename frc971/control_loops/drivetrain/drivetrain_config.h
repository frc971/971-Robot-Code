#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_CONSTANTS_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_CONSTANTS_H_

#include <functional>

#include "frc971/shifter_hall_effect.h"
#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

enum class ShifterType : int32_t {
  HALL_EFFECT_SHIFTER = 0,  // Detect when inbetween gears.
  SIMPLE_SHIFTER = 1,       // Switch gears without speedmatch logic.
  NO_SHIFTER = 2,           // Only one gear ratio.
};

enum class LoopType : int32_t {
  OPEN_LOOP = 0,    // Only use open loop logic.
  CLOSED_LOOP = 1,  // Add in closed loop calculation.
};

struct DrivetrainConfig {
  // Shifting method we are using.
  ShifterType shifter_type;

  // Type of loop to use.
  LoopType loop_type;

  // Polydrivetrain functions returning various controller loops with plants.
  ::std::function<StateFeedbackLoop<4, 2, 2>()> make_drivetrain_loop;
  ::std::function<StateFeedbackLoop<2, 2, 2>()> make_v_drivetrain_loop;
  ::std::function<StateFeedbackLoop<7, 2, 3>()> make_kf_drivetrain_loop;

  double dt;            // Control loop time step.
  double robot_radius;  // Robot radius, in meters.
  double wheel_radius;  // Wheel radius, in meters.
  double v;             // Motor velocity constant.

  // Gear ratios, from wheel to motor shaft.
  double high_gear_ratio;
  double low_gear_ratio;

  // Hall effect constants. Unused if not applicable to shifter type.
  constants::ShifterHallEffect left_drive;
  constants::ShifterHallEffect right_drive;

  // Variable that holds the default gear ratio. We use this in ZeroOutputs().
  // (ie. true means high gear is default).
  bool default_high_gear;

  double down_offset;

  double wheel_non_linearity;

  double quickturn_wheel_multiplier;

  // Converts the robot state to a linear distance position, velocity.
  static Eigen::Matrix<double, 2, 1> LeftRightToLinear(
      const Eigen::Matrix<double, 7, 1> &left_right) {
    Eigen::Matrix<double, 2, 1> linear;
    linear(0, 0) = (left_right(0, 0) + left_right(2, 0)) / 2.0;
    linear(1, 0) = (left_right(1, 0) + left_right(3, 0)) / 2.0;
    return linear;
  }
  // Converts the robot state to an anglular distance, velocity.
  Eigen::Matrix<double, 2, 1> LeftRightToAngular(
      const Eigen::Matrix<double, 7, 1> &left_right) const {
    Eigen::Matrix<double, 2, 1> angular;
    angular(0, 0) =
        (left_right(2, 0) - left_right(0, 0)) / (this->robot_radius * 2.0);
    angular(1, 0) =
        (left_right(3, 0) - left_right(1, 0)) / (this->robot_radius * 2.0);
    return angular;
  }

  // Converts the linear and angular position, velocity to the top 4 states of
  // the robot state.
  Eigen::Matrix<double, 4, 1> AngularLinearToLeftRight(
      const Eigen::Matrix<double, 2, 1> &linear,
      const Eigen::Matrix<double, 2, 1> &angular) const {
    Eigen::Matrix<double, 2, 1> scaled_angle =
        angular * this->robot_radius;
    Eigen::Matrix<double, 4, 1> state;
    state(0, 0) = linear(0, 0) - scaled_angle(0, 0);
    state(1, 0) = linear(1, 0) - scaled_angle(1, 0);
    state(2, 0) = linear(0, 0) + scaled_angle(0, 0);
    state(3, 0) = linear(1, 0) + scaled_angle(1, 0);
    return state;
  }
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_CONSTANTS_H_
