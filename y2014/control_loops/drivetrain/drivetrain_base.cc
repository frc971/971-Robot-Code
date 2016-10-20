#include "y2014/control_loops/drivetrain/drivetrain_base.h"

#include "frc971/control_loops/drivetrain/drivetrain_config.h"

#include "y2014/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2014/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"
#include "y2014/control_loops/drivetrain/kalman_drivetrain_motor_plant.h"
#include "y2014/constants.h"

using ::frc971::control_loops::drivetrain::DrivetrainConfig;

namespace y2014 {
namespace control_loops {

const DrivetrainConfig &GetDrivetrainConfig() {
  // TODO(austin): Switch over to using the profile.
  static DrivetrainConfig kDrivetrainConfig{
      ::frc971::control_loops::drivetrain::ShifterType::HALL_EFFECT_SHIFTER,
      ::frc971::control_loops::drivetrain::LoopType::CLOSED_LOOP,

      ::y2014::control_loops::drivetrain::MakeDrivetrainLoop,
      ::y2014::control_loops::drivetrain::MakeVelocityDrivetrainLoop,
      ::y2014::control_loops::drivetrain::MakeKFDrivetrainLoop,

      drivetrain::kDt,
      drivetrain::kRobotRadius,
      drivetrain::kWheelRadius,
      drivetrain::kV,

      constants::GetValues().high_gear_ratio,
      constants::GetValues().low_gear_ratio,
      constants::GetValues().left_drive,
      constants::GetValues().right_drive,
      true,
      0,
      0.25,
      1.0};

  return kDrivetrainConfig;
};

}  // namespace control_loops
}  // namespace y2014
