#ifndef Y2024_BOT3_CONSTANTS_H_
#define Y2024_BOT3_CONSTANTS_H_

#include <array>
#include <cmath>
#include <cstdint>

#include "frc971/constants.h"
#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "frc971/zeroing/absolute_encoder.h"
#include "frc971/zeroing/pot_and_absolute_encoder.h"
#include "y2024_bot3/constants/constants_generated.h"
#include "y2024_bot3/control_loops/drivetrain/rotation_plant.h"
#include "y2024_bot3/control_loops/superstructure/arm/arm_plant.h"

namespace y2024_bot3::constants {

constexpr uint16_t kThirdRobotTeamNumber = 9971;

struct Values {
  static const int kSuperstructureCANWriterPriority = 35;

  struct PotAndAbsEncoderConstants {
    ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
        ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator>
        subsystem_params;
    double potentiometer_offset;
  };

  struct AbsoluteEncoderConstants {
    ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
        ::frc971::zeroing::AbsoluteEncoderZeroingEstimator>
        subsystem_params;
  };

  struct PotConstants {
    ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
        ::frc971::zeroing::RelativeEncoderZeroingEstimator>
        subsystem_params;
    double potentiometer_offset;
  };

  // todo: get the correct values for all these constants
  static constexpr double kIntakeRollerOutputRatio = (16.0 / 34.0);
  static constexpr double kArmOutputRatio =
      control_loops::superstructure::arm::kOutputRatio;

  static constexpr double kArmPotRatio() { return (12.0 / 48.0); }

  static constexpr double kArmEncoderCountsPerRevolution() { return 4096.0; }

  static constexpr double kArmEncoderRatio() { return (1.0 / 4.0); }

  static constexpr double kArmPotRadiansPerVolt() {
    return kArmPotRatio() * (10.0 /*turns*/ / 5.0 /*volts*/) *
           (2 * M_PI /*radians*/);
  }

  static constexpr double kRotationModuleRatio =
      control_loops::drivetrain::kOutputRatio;

  static constexpr double kTranslationModuleRatio() {
    return (12.0 / 54.0 * 38.0 / 16.0 * 15.0 / 45.0) * 1.8 * 0.0254;
  }

  // note: there is one mag encoder per swerve module, and they measure only
  // rotation therefore, these encoder ratios are only rotation ratios
  static constexpr double kDrivetrainCyclesPerRevolution() { return 512.0; }
  static constexpr double kDrivetrainEncoderCountsPerRevolution() {
    return kDrivetrainCyclesPerRevolution() * 4;
  }

  static constexpr double kDrivetrainEncoderRatio() { return 1.0; }

  static constexpr double kMaxDrivetrainEncoderPulsesPerSecond() {
    return control_loops::drivetrain::kFreeSpeed / (2.0 * M_PI) *
           control_loops::drivetrain::kOutputRatio /
           constants::Values::kDrivetrainEncoderRatio() *
           kDrivetrainEncoderCountsPerRevolution();
  }

  static constexpr double kMaxArmEncoderPulsesPerSecond() {
    return control_loops::superstructure::arm::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::arm::kOutputRatio /
           kArmEncoderRatio() * kArmEncoderCountsPerRevolution();
  }
};

// Creates and returns a Values instance for the constants.
// Should be called before realtime because this allocates memory.
// Only the first call to either of these will be used.
constants::Values MakeValues(uint16_t team);

// Calls MakeValues with aos::network::GetTeamNumber()
constants::Values MakeValues();

}  // namespace y2024_bot3::constants

#endif  // Y2024_BOT3_CONSTANTS_H_
