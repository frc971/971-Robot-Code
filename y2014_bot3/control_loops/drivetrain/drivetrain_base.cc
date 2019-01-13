#include "y2014_bot3/control_loops/drivetrain/drivetrain_base.h"

#include <chrono>

#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "y2014_bot3/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2014_bot3/control_loops/drivetrain/hybrid_velocity_drivetrain.h"
#include "y2014_bot3/control_loops/drivetrain/kalman_drivetrain_motor_plant.h"
#include "y2014_bot3/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"

using ::frc971::control_loops::drivetrain::DrivetrainConfig;

namespace chrono = ::std::chrono;

namespace y2014_bot3 {
namespace control_loops {
namespace drivetrain {

using ::frc971::constants::ShifterHallEffect;

const ShifterHallEffect kThreeStateDriveShifter{0.0, 0.0, 0.25, 0.75};

const DrivetrainConfig<double> &GetDrivetrainConfig() {
  static DrivetrainConfig<double> kDrivetrainConfig{
      ::frc971::control_loops::drivetrain::ShifterType::SIMPLE_SHIFTER,
      ::frc971::control_loops::drivetrain::LoopType::CLOSED_LOOP,
      ::frc971::control_loops::drivetrain::GyroType::SPARTAN_GYRO,
      ::frc971::control_loops::drivetrain::IMUType::IMU_X,

      ::y2014_bot3::control_loops::drivetrain::MakeDrivetrainLoop,
      ::y2014_bot3::control_loops::drivetrain::MakeVelocityDrivetrainLoop,
      ::y2014_bot3::control_loops::drivetrain::MakeKFDrivetrainLoop,
      ::y2014_bot3::control_loops::drivetrain::MakeHybridVelocityDrivetrainLoop,

      chrono::duration_cast<chrono::nanoseconds>(
          chrono::duration<double>(drivetrain::kDt)),
      drivetrain::kRobotRadius, drivetrain::kWheelRadius, drivetrain::kV,

      drivetrain::kHighGearRatio, drivetrain::kLowGearRatio, drivetrain::kJ,
      drivetrain::kMass,

      // No shifter sensors, so we could put anything for the things below.
      kThreeStateDriveShifter, kThreeStateDriveShifter,
      false /* default_high_gear */, 0.0, 0.60 /* wheel_non_linearity */,
      0.60 /* quickturn_wheel_multiplier */, 0.7 /* wheel_multiplier */,
  };

  return kDrivetrainConfig;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2014_bot3
