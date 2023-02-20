#ifndef Y2023_CONSTANTS_H_
#define Y2023_CONSTANTS_H_

#include <array>
#include <cmath>
#include <cstdint>

#include "frc971/constants.h"
#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "y2023/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2023/control_loops/superstructure/arm/arm_constants.h"

namespace y2023 {
namespace constants {

constexpr uint16_t kCompTeamNumber = 971;
constexpr uint16_t kPracticeTeamNumber = 9971;
constexpr uint16_t kCodingRobotTeamNumber = 7971;

struct Values {
  static const int kZeroingSampleSize = 200;

  static const int kDrivetrainWriterPriority = 35;
  static const int kDrivetrainTxPriority = 36;
  static const int kDrivetrainRxPriority = 36;

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

  static constexpr double kDrivetrainSupplyCurrentLimit() { return 35.0; }
  static constexpr double kDrivetrainStatorCurrentLimit() { return 40.0; }

  static double DrivetrainEncoderToMeters(int32_t in) {
    return ((static_cast<double>(in) /
             kDrivetrainEncoderCountsPerRevolution()) *
            (2.0 * M_PI)) *
           kDrivetrainEncoderRatio() * control_loops::drivetrain::kWheelRadius;
  }

  static double DrivetrainCANEncoderToMeters(double rotations) {
    return (rotations * (2.0 * M_PI)) *
           control_loops::drivetrain::kHighOutputRatio *
           control_loops::drivetrain::kWheelRadius;
  }
  static constexpr double kProximalEncoderCountsPerRevolution() {
    return 4096.0;
  }
  static constexpr double kProximalEncoderRatio() { return (15.0 / 95.0); }
  static constexpr double kMaxProximalEncoderPulsesPerSecond() {
    return control_loops::superstructure::arm::kArmConstants.free_speed /
           (2.0 * M_PI) / control_loops::superstructure::arm::kArmConstants.g1 /
           kProximalEncoderRatio() * kProximalEncoderCountsPerRevolution();
  }
  static constexpr double kProximalPotRatio() {
    return (24.0 / 36.0) * (24.0 / 58.0) * (15.0 / 95.0);
  }

  static constexpr double kDistalEncoderCountsPerRevolution() { return 4096.0; }
  static constexpr double kDistalEncoderRatio() { return (15.0 / 95.0); }
  static constexpr double kMaxDistalEncoderPulsesPerSecond() {
    return control_loops::superstructure::arm::kArmConstants.free_speed /
           (2.0 * M_PI) / control_loops::superstructure::arm::kArmConstants.g2 /
           kDistalEncoderRatio() * kProximalEncoderCountsPerRevolution();
  }
  static constexpr double kDistalPotRatio() {
    return (24.0 / 36.0) * (18.0 / 66.0) * (15.0 / 95.0);
  }

  struct PotConstants {
    ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
        ::frc971::zeroing::RelativeEncoderZeroingEstimator>
        subsystem_params;
    double potentiometer_offset;
  };

  struct PotAndAbsEncoderConstants {
    ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
        ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator>
        subsystem_params;
    double potentiometer_offset;
  };

  struct ArmJointConstants {
    ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants zeroing;
    double potentiometer_offset;
  };

  ArmJointConstants arm_proximal;
  ArmJointConstants arm_distal;
};

// Creates and returns a Values instance for the constants.
// Should be called before realtime because this allocates memory.
// Only the first call to either of these will be used.
Values MakeValues(uint16_t team);

// Calls MakeValues with aos::network::GetTeamNumber()
Values MakeValues();

}  // namespace constants
}  // namespace y2023

#endif  // Y2023_CONSTANTS_H_
