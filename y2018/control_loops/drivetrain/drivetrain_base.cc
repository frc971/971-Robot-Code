#include "y2018/control_loops/drivetrain/drivetrain_base.h"

#include <chrono>

#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "y2018/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2018/control_loops/drivetrain/hybrid_velocity_drivetrain.h"
#include "y2018/control_loops/drivetrain/kalman_drivetrain_motor_plant.h"
#include "y2018/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"

using ::frc971::control_loops::drivetrain::DrivetrainConfig;

namespace chrono = ::std::chrono;

namespace y2018 {
namespace control_loops {
namespace drivetrain {

using ::frc971::constants::ShifterHallEffect;

const ShifterHallEffect kThreeStateDriveShifter{0.0, 0.0, 0.25, 0.75};

const DrivetrainConfig<double> &GetDrivetrainConfig() {
  static DrivetrainConfig<double> kDrivetrainConfig{
      ::frc971::control_loops::drivetrain::ShifterType::SIMPLE_SHIFTER,
      ::frc971::control_loops::drivetrain::LoopType::CLOSED_LOOP,
      ::frc971::control_loops::drivetrain::GyroType::IMU_Z_GYRO,
      ::frc971::control_loops::drivetrain::IMUType::IMU_Y,

      drivetrain::MakeDrivetrainLoop,
      drivetrain::MakeVelocityDrivetrainLoop,
      drivetrain::MakeKFDrivetrainLoop,
      drivetrain::MakeHybridVelocityDrivetrainLoop,

      chrono::duration_cast<chrono::nanoseconds>(
          chrono::duration<double>(drivetrain::kDt)),
      drivetrain::kRobotRadius, drivetrain::kWheelRadius, drivetrain::kV,

      drivetrain::kHighGearRatio, drivetrain::kLowGearRatio,
      drivetrain::kJ,
      drivetrain::kMass,
      kThreeStateDriveShifter, kThreeStateDriveShifter,
      true /* default_high_gear */, 0 /* down_offset if using constants use
                                   constants::GetValues().down_error */,
      0.8 /* wheel_non_linearity */, 1.2 /* quickturn_wheel_multiplier */,
      1.5 /* wheel_multiplier */,
  };

  return kDrivetrainConfig;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2018
