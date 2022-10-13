#ifndef Y2022_BOT3_CONSTANTS_H_
#define Y2022_BOT3_CONSTANTS_H_

#include <array>
#include <cmath>
#include <cstdint>

#include "frc971/constants.h"
#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "frc971/shooter_interpolation/interpolation.h"
#include "y2022_bot3/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2022_bot3/control_loops/superstructure/climber/climber_plant.h"
#include "y2022_bot3/control_loops/superstructure/intake/intake_plant.h"

using ::frc971::shooter_interpolation::InterpolationTable;

namespace y2022_bot3 {
namespace constants {

struct Values {
  static const int kZeroingSampleSize = 200;

  static constexpr double kDrivetrainCyclesPerRevolution() { return 512.0; }
  static constexpr double kDrivetrainEncoderCountsPerRevolution() {
    return kDrivetrainCyclesPerRevolution() * 4;
  }
  static constexpr double kDrivetrainEncoderRatio() { return 1.0; }
  static constexpr double kMaxDrivetrainEncoderPulsesPerSecond() {
    return control_loops::drivetrain::kFreeSpeed / (2.0 * M_PI) *
           control_loops::drivetrain::kHighOutputRatio /
           constants::Values::kDrivetrainEncoderRatio() *
           kDrivetrainEncoderCountsPerRevolution();
  }

  static double DrivetrainEncoderToMeters(int32_t in) {
    return ((static_cast<double>(in) /
             kDrivetrainEncoderCountsPerRevolution()) *
            (2.0 * M_PI)) *
           kDrivetrainEncoderRatio() * control_loops::drivetrain::kWheelRadius;
  }

  struct PotAndAbsEncoderConstants {
    ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
        ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator>
        subsystem_params;
    double potentiometer_offset;
  };

  static constexpr double kIntakeRollerSupplyCurrentLimit() { return 40.0; }
  static constexpr double kIntakeRollerStatorCurrentLimit() { return 60.0; }

  // Intake
  // TODO (Logan): Constants need to be tuned
  static constexpr double kIntakeEncoderCountsPerRevolution() { return 4096.0; }

  static constexpr double kIntakeEncoderRatio() {
    return (16.0 / 64.0) * (18.0 / 62.0);
  }

  static constexpr double kIntakePotRatio() { return 16.0 / 64.0; }

  static constexpr double kMaxIntakeEncoderPulsesPerSecond() {
    return control_loops::superstructure::intake::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::intake::kOutputRatio /
           kIntakeEncoderRatio() * kIntakeEncoderCountsPerRevolution();
  }
  PotAndAbsEncoderConstants intake;

  static constexpr ::frc971::constants::Range kIntakeRange() {
    return ::frc971::constants::Range{
        .lower_hard = -0.85,  // Back Hard
        .upper_hard = 1.85,   // Front Hard
        .lower = -0.400,      // Back Soft
        .upper = 1.57         // Front Soft
    };
  }

  // Climber
  // TODO (Logan): Constants need to be tuned
  static constexpr double kClimberEncoderCountsPerRevolution() {
    return 4096.0;
  }

  static constexpr double kClimberEncoderRatio() {
    return (16.0 / 64.0) * (18.0 / 62.0);
  }

  static constexpr double kClimberEncoderMetersPerRevolution() { return 1.0; }

  static constexpr double kClimberPotMetersPerRevolution() { return 0.125; }

  static constexpr double kClimberPotRatio() { return 16.0 / 64.0; }

  static constexpr double kMaxClimberEncoderPulsesPerSecond() {
    return control_loops::superstructure::climber::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::climber::kOutputRatio /
           kClimberEncoderRatio() * kClimberEncoderCountsPerRevolution();
  }
  PotAndAbsEncoderConstants climber_left;
  PotAndAbsEncoderConstants climber_right;

  static constexpr ::frc971::constants::Range kClimberRange() {
    return ::frc971::constants::Range{
        .lower_hard = -0.01,  // Back Hard
        .upper_hard = 0.59,   // Front Hard
        .lower = 0.003,       // Back Soft
        .upper = 0.555        // Front Soft
    };
  }
};

// Creates and returns a Values instance for the constants.
// Should be called before realtime because this allocates memory.
// Only the first call to either of these will be used.
Values MakeValues(uint16_t team);

// Calls MakeValues with aos::network::GetTeamNumber()
Values MakeValues();

}  // namespace constants
}  // namespace y2022_bot3

#endif  // Y2022_BOT3_CONSTANTS_H_
