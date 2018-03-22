#include "y2012/control_loops/drivetrain/drivetrain_base.h"

#include "frc971/control_loops/drivetrain/drivetrain_config.h"

#include "frc971/control_loops/state_feedback_loop.h"
#include "y2012/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2012/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"
#include "y2012/control_loops/drivetrain/kalman_drivetrain_motor_plant.h"

using ::frc971::control_loops::drivetrain::DrivetrainConfig;

namespace y2012 {
namespace control_loops {
namespace drivetrain {

using ::frc971::constants::ShifterHallEffect;

const ShifterHallEffect kThreeStateDriveShifter{0.0, 0.0, 0.25, 0.75};

const DrivetrainConfig &GetDrivetrainConfig() {
  static DrivetrainConfig kDrivetrainConfig{
      ::frc971::control_loops::drivetrain::ShifterType::NO_SHIFTER,
      ::frc971::control_loops::drivetrain::LoopType::CLOSED_LOOP,
      ::frc971::control_loops::drivetrain::GyroType::SPARTAN_GYRO,
      ::frc971::control_loops::drivetrain::IMUType::IMU_X,

      ::y2012::control_loops::drivetrain::MakeDrivetrainLoop,
      ::y2012::control_loops::drivetrain::MakeVelocityDrivetrainLoop,
      ::y2012::control_loops::drivetrain::MakeKFDrivetrainLoop,

      drivetrain::kDt,
      drivetrain::kRobotRadius,
      drivetrain::kWheelRadius,
      drivetrain::kV,

      drivetrain::kHighGearRatio,
      drivetrain::kLowGearRatio,
      kThreeStateDriveShifter,
      kThreeStateDriveShifter,
      true /* default_high_gear */,
      0.0,
      0.4 /* wheel_non_linearity */,
      1.0 /* quickturn_wheel_multiplier */,
      1.0 /* wheel_multiplier */};

  return kDrivetrainConfig;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2012
