#include "y2016/control_loops/drivetrain/drivetrain_base.h"

#include "frc971/control_loops/drivetrain/drivetrain_config.h"

#include "y2016/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2016/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"
#include "y2016/control_loops/drivetrain/kalman_drivetrain_motor_plant.h"
#include "y2016/constants.h"

using ::frc971::control_loops::drivetrain::DrivetrainConfig;

namespace y2016 {
namespace control_loops {

const DrivetrainConfig &GetDrivetrainConfig() {
  static DrivetrainConfig kDrivetrainConfig{
      ::frc971::control_loops::drivetrain::ShifterType::HALL_EFFECT_SHIFTER,

      ::y2016::control_loops::drivetrain::MakeDrivetrainLoop,
      ::y2016::control_loops::drivetrain::MakeVelocityDrivetrainLoop,
      ::y2016::control_loops::drivetrain::MakeKFDrivetrainLoop,

      drivetrain::kDt,
      drivetrain::kStallTorque,
      drivetrain::kStallCurrent,
      drivetrain::kFreeSpeedRPM,
      drivetrain::kFreeCurrent,
      drivetrain::kJ,
      drivetrain::kMass,
      drivetrain::kRobotRadius,
      drivetrain::kWheelRadius,
      drivetrain::kR,
      drivetrain::kV,
      drivetrain::kT,

      constants::GetValues().turn_width,
      constants::GetValues().high_gear_ratio,
      constants::GetValues().low_gear_ratio,
      constants::GetValues().left_drive,
      constants::GetValues().right_drive};

  return kDrivetrainConfig;
};

}  // namespace control_loops
}  // namespace y2016
