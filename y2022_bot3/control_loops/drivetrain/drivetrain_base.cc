#include "y2022_bot3/control_loops/drivetrain/drivetrain_base.h"

#include <chrono>

#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "y2022_bot3/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2022_bot3/control_loops/drivetrain/hybrid_velocity_drivetrain.h"
#include "y2022_bot3/control_loops/drivetrain/kalman_drivetrain_motor_plant.h"
#include "y2022_bot3/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"

using ::frc971::control_loops::drivetrain::DownEstimatorConfig;
using ::frc971::control_loops::drivetrain::DrivetrainConfig;

namespace chrono = ::std::chrono;

namespace y2022_bot3 {
namespace control_loops {
namespace drivetrain {

using ::frc971::constants::ShifterHallEffect;

const ShifterHallEffect kThreeStateDriveShifter{0.0, 0.0, 0.25, 0.75};

const DrivetrainConfig<double> &GetDrivetrainConfig() {
  // Yaw of the IMU relative to the robot frame.
  static constexpr double kImuYaw = 0.0;
  static DrivetrainConfig<double> kDrivetrainConfig{
      ::frc971::control_loops::drivetrain::ShifterType::SIMPLE_SHIFTER,
      ::frc971::control_loops::drivetrain::LoopType::CLOSED_LOOP,
      ::frc971::control_loops::drivetrain::GyroType::SPARTAN_GYRO,
      ::frc971::control_loops::drivetrain::IMUType::IMU_FLIPPED_X,

      drivetrain::MakeDrivetrainLoop,
      drivetrain::MakeVelocityDrivetrainLoop,
      drivetrain::MakeKFDrivetrainLoop,
      drivetrain::MakeHybridVelocityDrivetrainLoop,

      chrono::duration_cast<chrono::nanoseconds>(
          chrono::duration<double>(drivetrain::kDt)),
      drivetrain::kRobotRadius,
      drivetrain::kWheelRadius,
      drivetrain::kV,

      drivetrain::kHighGearRatio,
      drivetrain::kLowGearRatio,
      drivetrain::kJ,
      drivetrain::kMass,
      kThreeStateDriveShifter,
      kThreeStateDriveShifter,
      true /* default_high_gear */,
      0 /* down_offset if using constants use
     constants::GetValues().down_error */
      ,
      0.7 /* wheel_non_linearity */,
      1.2 /* quickturn_wheel_multiplier */,
      1.2 /* wheel_multiplier */,
      true /*pistol_grip_shift_enables_line_follow*/,
      (Eigen::Matrix<double, 3, 3>() << std::cos(kImuYaw), -std::sin(kImuYaw),
       0.0, std::sin(kImuYaw), std::cos(kImuYaw), 0.0, 0.0, 0.0, 1.0)
          .finished(),
      false /*is_simulated*/,
      DownEstimatorConfig{.gravity_threshold = 0.015,
                          .do_accel_corrections = 1000}};

  return kDrivetrainConfig;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2022_bot3
