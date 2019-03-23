#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_CONSTANTS_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_CONSTANTS_H_

#include <functional>

#if defined(__linux__)
#include "frc971/control_loops/hybrid_state_feedback_loop.h"
#endif
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/shifter_hall_effect.h"

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

enum class GyroType : int32_t {
  SPARTAN_GYRO = 0, // Use the gyro on the spartan board.
  IMU_X_GYRO = 1,   // Use the x-axis of the gyro on the IMU.
  IMU_Y_GYRO = 2,   // Use the y-axis of the gyro on the IMU.
  IMU_Z_GYRO = 3,   // Use the z-axis of the gyro on the IMU.
  FLIPPED_SPARTAN_GYRO = 4, // Use the gyro on the spartan board.
  FLIPPED_IMU_Z_GYRO = 5,   // Use the flipped z-axis of the gyro on the IMU.
};

enum class IMUType : int32_t {
  IMU_X = 0,          // Use the x-axis of the IMU.
  IMU_Y = 1,          // Use the y-axis of the IMU.
  IMU_FLIPPED_X = 2,  // Use the flipped x-axis of the IMU.
};

template <typename Scalar = double>
struct DrivetrainConfig {
  // Shifting method we are using.
  ShifterType shifter_type;

  // Type of loop to use.
  LoopType loop_type;

  // Type of gyro to use.
  GyroType gyro_type;

  // Type of IMU to use.
  IMUType imu_type;

  // Polydrivetrain functions returning various controller loops with plants.
  ::std::function<StateFeedbackLoop<4, 2, 2, Scalar>()> make_drivetrain_loop;
  ::std::function<StateFeedbackLoop<2, 2, 2, Scalar>()> make_v_drivetrain_loop;
  ::std::function<StateFeedbackLoop<7, 2, 4, Scalar>()> make_kf_drivetrain_loop;
#if defined(__linux__)
  ::std::function<
      StateFeedbackLoop<2, 2, 2, Scalar, StateFeedbackHybridPlant<2, 2, 2>,
                        HybridKalman<2, 2, 2>>()>
      make_hybrid_drivetrain_velocity_loop;
#endif

  ::std::chrono::nanoseconds dt;  // Control loop time step.
  Scalar robot_radius;            // Robot radius, in meters.
  Scalar wheel_radius;            // Wheel radius, in meters.
  Scalar v;                       // Motor velocity constant.

  // Gear ratios, from wheel to motor shaft.
  Scalar high_gear_ratio;
  Scalar low_gear_ratio;

  // Moment of inertia and mass.
  Scalar J;
  Scalar mass;

  // Hall effect constants. Unused if not applicable to shifter type.
  constants::ShifterHallEffect left_drive;
  constants::ShifterHallEffect right_drive;

  // Variable that holds the default gear ratio. We use this in ZeroOutputs().
  // (ie. true means high gear is default).
  bool default_high_gear;

  Scalar down_offset;

  Scalar wheel_non_linearity;

  Scalar quickturn_wheel_multiplier;

  Scalar wheel_multiplier;

  // Whether the shift button on the pistol grip enables line following mode.
  bool pistol_grip_shift_enables_line_follow = false;

  // Converts the robot state to a linear distance position, velocity.
  static Eigen::Matrix<Scalar, 2, 1> LeftRightToLinear(
      const Eigen::Matrix<Scalar, 7, 1> &left_right) {
    Eigen::Matrix<Scalar, 2, 1> linear;
    linear(0, 0) = (left_right(0, 0) + left_right(2, 0)) / 2.0;
    linear(1, 0) = (left_right(1, 0) + left_right(3, 0)) / 2.0;
    return linear;
  }
  // Converts the robot state to an anglular distance, velocity.
  Eigen::Matrix<Scalar, 2, 1> LeftRightToAngular(
      const Eigen::Matrix<Scalar, 7, 1> &left_right) const {
    Eigen::Matrix<Scalar, 2, 1> angular;
    angular(0, 0) =
        (left_right(2, 0) - left_right(0, 0)) / (this->robot_radius * 2.0);
    angular(1, 0) =
        (left_right(3, 0) - left_right(1, 0)) / (this->robot_radius * 2.0);
    return angular;
  }

  Eigen::Matrix<Scalar, 2, 2> Tlr_to_la() const {
    return (::Eigen::Matrix<Scalar, 2, 2>() << 0.5, 0.5,
            -1.0 / (2 * robot_radius), 1.0 / (2 * robot_radius)).finished();
  }

  Eigen::Matrix<Scalar, 2, 2> Tla_to_lr() const {
    return Tlr_to_la().inverse();
  }

  // Converts the linear and angular position, velocity to the top 4 states of
  // the robot state.
  Eigen::Matrix<Scalar, 4, 1> AngularLinearToLeftRight(
      const Eigen::Matrix<Scalar, 2, 1> &linear,
      const Eigen::Matrix<Scalar, 2, 1> &angular) const {
    Eigen::Matrix<Scalar, 2, 1> scaled_angle =
        angular * this->robot_radius;
    Eigen::Matrix<Scalar, 4, 1> state;
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
