#ifndef Y2022_CONSTANTS_H_
#define Y2022_CONSTANTS_H_

#include <array>
#include <cmath>
#include <cstdint>

#include "frc971/constants.h"
#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "y2022/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2022/control_loops/superstructure/climber/climber_plant.h"
#include "y2022/control_loops/superstructure/intake/intake_plant.h"
#include "y2022/control_loops/superstructure/turret/turret_plant.h"

namespace y2022 {
namespace constants {

struct Values {
  static const int kZeroingSampleSize = 200;

  static constexpr double kDrivetrainCyclesPerRevolution() { return 512.0; }
  static constexpr double kDrivetrainEncoderCountsPerRevolution() {
    return kDrivetrainCyclesPerRevolution() * 4;
  }
  static constexpr double kDrivetrainEncoderRatio() {
    return (14.0 / 54.0) * (22.0 / 56.0);
  }
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

  // Climber
  static constexpr ::frc971::constants::Range kClimberRange() {
    return ::frc971::constants::Range{
        .lower_hard = -0.01, .upper_hard = 0.6, .lower = 0.0, .upper = 0.5};
  }
  static constexpr double kClimberPotMetersPerRevolution() {
    return 22 * 0.25 * 0.0254;
  }
  static constexpr double kClimberPotRatio() { return 1.0; }

  struct PotConstants {
    ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
        ::frc971::zeroing::RelativeEncoderZeroingEstimator>
        subsystem_params;
    double potentiometer_offset;
  };

  PotConstants climber;

  // Intake
  // two encoders with same gear ratio for intake
  static constexpr double kIntakeEncoderCountsPerRevolution() { return 4096.0; }

  static constexpr double kIntakeEncoderRatio() {
    return (16.0 / 64.0) * (20.0 / 50.0);
  }

  static constexpr double kIntakePotRatio() { return 16.0 / 64.0; }

  static constexpr double kMaxIntakeEncoderPulsesPerSecond() {
    return control_loops::superstructure::intake::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::intake::kOutputRatio /
           kIntakeEncoderRatio() * kIntakeEncoderCountsPerRevolution();
  }

  struct PotAndAbsEncoderConstants {
    ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
        ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator>
        subsystem_params;
    double potentiometer_offset;
  };

  PotAndAbsEncoderConstants intake_front;
  PotAndAbsEncoderConstants intake_back;

  // TODO (Yash): Constants need to be tuned
  static constexpr ::frc971::constants::Range kIntakeRange() {
    return ::frc971::constants::Range{
        .lower_hard = -0.5,         // Back Hard
        .upper_hard = 2.85 + 0.05,  // Front Hard
        .lower = -0.300,            // Back Soft
        .upper = 2.725              // Front Soft
    };
  }

  // Intake rollers
  static constexpr double kIntakeRollerSupplyCurrentLimit() { return 40.0; }
  static constexpr double kIntakeRollerStatorCurrentLimit() { return 60.0; }

  // Turret
  PotAndAbsEncoderConstants turret;

  // TODO (Yash): Constants need to be tuned
  static constexpr ::frc971::constants::Range kTurretRange() {
    return ::frc971::constants::Range{
        .lower_hard = -3.45,  // Back Hard
        .upper_hard = 3.45,   // Front Hard
        .lower = -3.3,        // Back Soft
        .upper = 3.3          // Front Soft
    };
  }

  // Turret
  static constexpr double kTurretPotRatio() { return 27.0 / 110.0; }
  static constexpr double kTurretEncoderRatio() { return kTurretPotRatio(); }
  static constexpr double kTurretEncoderCountsPerRevolution() { return 4096.0; }

  static constexpr double kMaxTurretEncoderPulsesPerSecond() {
    return control_loops::superstructure::turret::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::turret::kOutputRatio /
           kTurretEncoderRatio() * kTurretEncoderCountsPerRevolution();
  }

  // Flipper arms
  static constexpr double kFlipperArmSupplyCurrentLimit() { return 30.0; }
  static constexpr double kFlipperArmStatorCurrentLimit() { return 40.0; }

  // TODO: (Griffin) this needs to be set
  static constexpr ::frc971::constants::Range kFlipperArmRange() {
    return ::frc971::constants::Range{
        .lower_hard = -0.01, .upper_hard = 0.6, .lower = 0.0, .upper = 0.5};
  }

  static constexpr double kFlipperArmsPotRatio() { return 16.0 / 36.0; }

  PotConstants flipper_arm_left;
  PotConstants flipper_arm_right;
};

// Creates and returns a Values instance for the constants.
// Should be called before realtime because this allocates memory.
// Only the first call to either of these will be used.
Values MakeValues(uint16_t team);

// Calls MakeValues with aos::network::GetTeamNumber()
Values MakeValues();

}  // namespace constants
}  // namespace y2022

#endif  // Y2022_CONSTANTS_H_
