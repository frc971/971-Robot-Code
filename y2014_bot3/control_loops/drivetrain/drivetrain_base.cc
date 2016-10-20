#include "y2014_bot3/control_loops/drivetrain/drivetrain_base.h"

#include "frc971/control_loops/drivetrain/drivetrain_config.h"

#include "frc971/control_loops/state_feedback_loop.h"
#include "y2014_bot3/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2014_bot3/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"
#include "y2014_bot3/control_loops/drivetrain/kalman_drivetrain_motor_plant.h"

using ::frc971::control_loops::drivetrain::DrivetrainConfig;

namespace y2014_bot3 {
namespace control_loops {
namespace drivetrain {

using ::frc971::constants::ShifterHallEffect;

const ShifterHallEffect kThreeStateDriveShifter{0.0, 0.0, 0.0, 0.0, 0.25, 0.75};

const DrivetrainConfig &GetDrivetrainConfig() {
  static DrivetrainConfig kDrivetrainConfig{
      ::frc971::control_loops::drivetrain::ShifterType::SIMPLE_SHIFTER,
      ::frc971::control_loops::drivetrain::LoopType::OPEN_LOOP,

      ::y2014_bot3::control_loops::drivetrain::MakeDrivetrainLoop,
      ::y2014_bot3::control_loops::drivetrain::MakeVelocityDrivetrainLoop,
      ::y2014_bot3::control_loops::drivetrain::MakeKFDrivetrainLoop,

      drivetrain::kDt,
      drivetrain::kRobotRadius,
      drivetrain::kWheelRadius,
      drivetrain::kV,

      drivetrain::kHighGearRatio,
      drivetrain::kLowGearRatio,

      // No shifter sensors, so we could put anything for the things below.
      kThreeStateDriveShifter,
      kThreeStateDriveShifter,
      false,
      0.0,
      0.60,
      0.60};

  return kDrivetrainConfig;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2014_bot3
