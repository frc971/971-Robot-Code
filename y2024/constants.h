#ifndef Y2024_CONSTANTS_H_
#define Y2024_CONSTANTS_H_

#include <array>
#include <cmath>
#include <cstdint>

#include "frc971/constants.h"
#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "frc971/zeroing/absolute_encoder.h"
#include "frc971/zeroing/pot_and_absolute_encoder.h"
#include "y2024/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2024/control_loops/superstructure/climber/climber_plant.h"
#include "y2024/control_loops/superstructure/intake_pivot/intake_pivot_plant.h"

namespace y2024::constants {

constexpr uint16_t kCompTeamNumber = 971;
constexpr uint16_t kPracticeTeamNumber = 9971;
constexpr uint16_t kCodingRobotTeamNumber = 7971;

struct Values {
  static const int kSuperstructureCANWriterPriority = 35;
  static const int kDrivetrainWriterPriority = 35;
  static const int kDrivetrainTxPriority = 36;
  static const int kDrivetrainRxPriority = 36;

  // TODO: These values will need to be changed for the 2024 robot.
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

  static double DrivetrainCANEncoderToMeters(double rotations) {
    return (rotations * (2.0 * M_PI)) *
           control_loops::drivetrain::kHighOutputRatio;
  }
  // TODO: (niko) add the gear ratios for the intake once we have them
  static constexpr double kIntakePivotEncoderCountsPerRevolution() {
    return 4096.0;
  }

  static constexpr double kIntakePivotEncoderRatio() {
    return (16.0 / 64.0) * (18.0 / 62.0);
  }

  static constexpr double kMaxIntakePivotEncoderPulsesPerSecond() {
    return control_loops::superstructure::intake_pivot::kFreeSpeed /
           (2.0 * M_PI) *
           control_loops::superstructure::intake_pivot::kOutputRatio /
           kIntakePivotEncoderRatio() *
           kIntakePivotEncoderCountsPerRevolution();
  }

  // TODO(Filip): Update climber values once we have them.
  static constexpr double kClimberEncoderCountsPerRevolution() {
    return 4096.0;
  }

  static constexpr double kClimberEncoderRatio() {
    return (16.0 / 64.0) * (18.0 / 62.0);
  }

  static constexpr double kClimberPotRatio() { return 16.0 / 64.0; }

  static constexpr double kClimberPotRadiansPerVolt() {
    return kClimberPotRatio() * (3.0 /*turns*/ / 5.0 /*volts*/) *
           (2 * M_PI /*radians*/);
  }

  static constexpr double kMaxClimberEncoderPulsesPerSecond() {
    return control_loops::superstructure::climber::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::climber::kOutputRatio /
           kClimberEncoderRatio() * kClimberEncoderCountsPerRevolution();
  }

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
};

// Creates and returns a Values instance for the constants.
// Should be called before realtime because this allocates memory.
// Only the first call to either of these will be used.
constants::Values MakeValues(uint16_t team);

// Calls MakeValues with aos::network::GetTeamNumber()
constants::Values MakeValues();

}  // namespace y2024::constants

#endif  // Y2024_CONSTANTS_H_
