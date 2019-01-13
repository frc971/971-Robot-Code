#include "y2014/control_loops/drivetrain/drivetrain_base.h"

#include <chrono>

#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "y2014/constants.h"
#include "y2014/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2014/control_loops/drivetrain/hybrid_velocity_drivetrain.h"
#include "y2014/control_loops/drivetrain/kalman_drivetrain_motor_plant.h"
#include "y2014/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"

using ::frc971::control_loops::drivetrain::DrivetrainConfig;

namespace chrono = ::std::chrono;

namespace y2014 {
namespace control_loops {

const DrivetrainConfig<double> &GetDrivetrainConfig() {
  // TODO(austin): Switch over to using the profile.
  static DrivetrainConfig<double> kDrivetrainConfig{
      ::frc971::control_loops::drivetrain::ShifterType::HALL_EFFECT_SHIFTER,
      ::frc971::control_loops::drivetrain::LoopType::CLOSED_LOOP,
      ::frc971::control_loops::drivetrain::GyroType::SPARTAN_GYRO,
      ::frc971::control_loops::drivetrain::IMUType::IMU_X,

      drivetrain::MakeDrivetrainLoop,
      drivetrain::MakeVelocityDrivetrainLoop,
      drivetrain::MakeKFDrivetrainLoop,
      drivetrain::MakeHybridVelocityDrivetrainLoop,

      chrono::duration_cast<chrono::nanoseconds>(
          chrono::duration<double>(drivetrain::kDt)),
      drivetrain::kRobotRadius,
      drivetrain::kWheelRadius,
      drivetrain::kV,

      constants::GetValues().high_gear_ratio,
      constants::GetValues().low_gear_ratio,
      drivetrain::kJ,
      drivetrain::kMass,
      constants::GetValues().left_drive.shifter_hall_effect,
      constants::GetValues().right_drive.shifter_hall_effect,
      true /* default_high_gear */,
      0,
      0.25 /* wheel_non_linearity */,
      1.0 /* quickturn_wheel_multiplier */,
      1.0 /* wheel_multiplier */,
  };

  return kDrivetrainConfig;
};

}  // namespace control_loops
}  // namespace y2014
