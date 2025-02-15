#ifndef y2025_CONSTANTS_H_
#define y2025_CONSTANTS_H_

#include <array>
#include <cmath>
#include <cstdint>

#include "frc971/constants.h"
#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "frc971/zeroing/absolute_encoder.h"
#include "frc971/zeroing/pot_and_absolute_encoder.h"
#include "y2025/constants/constants_generated.h"
#include "y2025/control_loops/drivetrain/rotation_plant.h"
#include "y2025/control_loops/superstructure/elevator/elevator_plant.h"
#include "y2025/control_loops/superstructure/pivot/pivot_plant.h"
#include "y2025/control_loops/superstructure/wrist/wrist_plant.h"

namespace y2025::constants {

constexpr uint16_t kRobotTeamNumber = 971;

struct Values {
  static const int kSuperstructureCANWriterPriority = 35;
  static const int kDrivetrainWriterPriority = 35;
  static const int kDrivetrainTxPriority = 36;
  static const int kDrivetrainRxPriority = 36;

  static constexpr double kRotationModuleRatio =
      control_loops::drivetrain::kOutputRatio;

  static constexpr double kWheelRadius = 1.8 * 0.0254;

  static constexpr double kTranslationModuleRatio() {
    return (12.0 / 54.0 * 38.0 / 16.0 * 15.0 / 45.0) * kWheelRadius;
  }

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

  static constexpr double kElevatorOutputRatio =
      control_loops::superstructure::elevator::kOutputRatio;

  static constexpr double kElevatorPotRatio() { return 1; }

  static constexpr double kElevatorPotMetersPerRevolution() {
    return (1.751) * 0.0254 * (2.0 * M_PI);
  }

  static constexpr double kElevatorEncoderMetersPerRadian() {
    return kElevatorPotMetersPerRevolution() / 2.0 / M_PI;
  }

  static constexpr double kElevatorPotMetersPerVolt() {
    return kElevatorPotRatio() * (10.0 /*turns*/ / 5.0 /*volts*/) *
           kElevatorPotMetersPerRevolution();
  }

  static constexpr double kElevatorEncoderCountsPerRevolution() {
    return 4096.0;
  }

  static constexpr double kElevatorEncoderRatio() { return 1.0; }

  static constexpr double kMaxElevatorEncoderPulsesPerSecond() {
    return control_loops::superstructure::elevator::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::elevator::kOutputRatio /
           kElevatorEncoderRatio() * kElevatorEncoderCountsPerRevolution();
  }
  struct PotAndAbsEncoderConstants {
    ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
        ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator>
        subsystem_params;
    double potentiometer_offset;
  };

  struct AbsEncoderConstants {
    ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
        ::frc971::zeroing::AbsoluteEncoderZeroingEstimator>
        subsystem_params;
  };

  static constexpr double kPivotOutputRatio =
      control_loops::superstructure::pivot::kOutputRatio;

  static constexpr double kPivotPotRatio() {
    return (38.0 / 20.0) * (10.0 / 110.0);
  }

  static constexpr double kPivotEncoderCountsPerRevolution() { return 4096.0; }

  static constexpr double kPivotEncoderRatio() { return (10.0 / 110.0); }

  static constexpr double kPivotPotRadiansPerVolt() {
    return kPivotPotRatio() * (10.0 /*turns*/ / 5.0 /*volts*/) *
           (2 * M_PI /*radians*/);
  }

  static constexpr double kMaxPivotEncoderPulsesPerSecond() {
    return control_loops::superstructure::pivot::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::pivot::kOutputRatio /
           kPivotEncoderRatio() * kPivotEncoderCountsPerRevolution();
  }

  // Wrist
  // TODO Set wrist constants
  static constexpr double kWristOutputRatio =
      control_loops::superstructure::wrist::kOutputRatio;
  static constexpr double kWristEncoderCountsPerRevolution() { return 4096.0; }
  static constexpr double kWristEncoderRatio() { return 1.0; }
  static constexpr double kMaxWristEncoderPulsesPerSecond() {
    return control_loops::superstructure::wrist::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::wrist::kOutputRatio /
           kWristEncoderRatio() * kWristEncoderCountsPerRevolution();
  }
  // TODO add a real value here
  static constexpr double kClimberOutputRatio = 1;
};

// Creates and returns a Values instance for the constants.
// Should be called before realtime because this allocates memory.
// Only the first call to either of these will be used.
constants::Values MakeValues(uint16_t team);

// Calls MakeValues with aos::network::GetTeamNumber()
constants::Values MakeValues();

}  // namespace y2025::constants

#endif  // y2025_CONSTANTS_H_
