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
#include "y2023/control_loops/superstructure/roll/roll_plant.h"
#include "y2023/control_loops/superstructure/wrist/wrist_plant.h"

namespace y2023 {
namespace constants {

constexpr uint16_t kCompTeamNumber = 971;
constexpr uint16_t kPracticeTeamNumber = 9971;
constexpr uint16_t kCodingRobotTeamNumber = 7971;

struct Values {
  static const int kZeroingSampleSize = 200;

  static const int kSuperstructureCANWriterPriority = 35;
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
  static constexpr double kDrivetrainStatorCurrentLimit() { return 60.0; }

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
           (2.0 * M_PI) / control_loops::superstructure::arm::kArmConstants.g0 /
           kProximalEncoderRatio() * kProximalEncoderCountsPerRevolution();
  }
  static constexpr double kProximalPotRatio() {
    return (36.0 / 24.0) * (15.0 / 95.0);
  }

  static constexpr double kProximalPotRadiansPerVolt() {
    return kProximalPotRatio() * (10.0 /*turns*/ / 5.0 /*volts*/) *
           (2 * M_PI /*radians*/);
  }

  static constexpr double kDistalEncoderCountsPerRevolution() { return 4096.0; }
  static constexpr double kDistalEncoderRatio() { return (15.0 / 96.0); }
  static constexpr double kMaxDistalEncoderPulsesPerSecond() {
    return control_loops::superstructure::arm::kArmConstants.free_speed /
           (2.0 * M_PI) / control_loops::superstructure::arm::kArmConstants.g1 /
           kDistalEncoderRatio() * kProximalEncoderCountsPerRevolution();
  }
  static constexpr double kDistalPotRatio() {
    return (36.0 / 24.0) * (15.0 / 96.0);
  }

  static constexpr double kDistalPotRadiansPerVolt() {
    return kDistalPotRatio() * (10.0 /*turns*/ / 5.0 /*volts*/) *
           (2 * M_PI /*radians*/);
  }

  // Roll joint
  static constexpr double kRollJointEncoderCountsPerRevolution() {
    return 4096.0;
  }

  static constexpr double kRollJointEncoderRatio() { return (18.0 / 48.0); }

  static constexpr double kRollJointPotRatio() { return (18.0 / 48.0); }

  static constexpr double kRollJointPotRadiansPerVolt() {
    return kRollJointPotRatio() * (3.0 /*turns*/ / 5.0 /*volts*/) *
           (2 * M_PI /*radians*/);
  }

  static constexpr double kMaxRollJointEncoderPulsesPerSecond() {
    return control_loops::superstructure::roll::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::roll::kOutputRatio /
           kRollJointEncoderRatio() * kRollJointEncoderCountsPerRevolution();
  }

  static constexpr ::frc971::constants::Range kRollJointRange() {
    return ::frc971::constants::Range{
        -1.05,  // Back Hard
        1.44,   // Front Hard
        -0.89,  // Back Soft
        1.26    // Front Soft
    };
  }

  // Wrist
  static constexpr double kWristEncoderCountsPerRevolution() { return 4096.0; }

  static constexpr double kCompWristEncoderRatio() { return 1.0; }
  static constexpr double kPracticeWristEncoderRatio() {
    return (24.0 / 36.0) * (36.0 / 60.0);
  }

  static constexpr double kMaxCompWristEncoderPulsesPerSecond() {
    return control_loops::superstructure::wrist::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::wrist::kOutputRatio /
           kCompWristEncoderRatio() * kWristEncoderCountsPerRevolution();
  }
  static constexpr double kMaxPracticeWristEncoderPulsesPerSecond() {
    return control_loops::superstructure::wrist::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::wrist::kOutputRatio /
           kPracticeWristEncoderRatio() * kWristEncoderCountsPerRevolution();
  }

  static constexpr ::frc971::constants::Range kCompWristRange() {
    return ::frc971::constants::Range{
        .lower_hard = -0.10,  // Back Hard
        .upper_hard = 4.90,   // Front Hard
        .lower = 0.0,         // Back Soft
        .upper = 4.0,         // Front Soft
    };
  }

  static constexpr ::frc971::constants::Range kPracticeWristRange() {
    return ::frc971::constants::Range{
        .lower_hard = -0.10,  // Back Hard
        .upper_hard = 2.30,   // Front Hard
        .lower = 0.0,         // Back Soft
        .upper = 2.2,         // Front Soft
    };
  }

  // Rollers
  static constexpr double kRollerSupplyCurrentLimit() { return 30.0; }
  static constexpr double kRollerStatorCurrentLimit() { return 100.0; }

  // Game object is fed into end effector for at least this time
  static constexpr std::chrono::milliseconds kExtraIntakingTime() {
    return std::chrono::seconds(2);
  }

  // Game object is spit from end effector for at least this time
  static constexpr std::chrono::milliseconds kExtraSpittingTime() {
    return std::chrono::seconds(1);
  }

  // if true, tune down all the arm constants for testing.
  static constexpr bool kArmGrannyMode() { return false; }

  // the operating voltage.
  static constexpr double kArmOperatingVoltage() {
    return kArmGrannyMode() ? 6.0 : 12.0;
  }
  static constexpr double kArmDt() { return 0.00505; }
  static constexpr std::chrono::nanoseconds kArmDtDuration() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(kArmDt()));
  }
  static constexpr double kArmAlpha0Max() {
    return kArmGrannyMode() ? 15.0 : 15.0;
  }
  static constexpr double kArmAlpha1Max() {
    return kArmGrannyMode() ? 10.0 : 10.0;
  }
  static constexpr double kArmAlpha2Max() {
    return kArmGrannyMode() ? 90.0 : 90.0;
  }

  static constexpr double kArmVMax() { return kArmGrannyMode() ? 4.0 : 9.5; }
  static constexpr double kArmPathlessVMax() { return 9.5; }
  static constexpr double kArmGotoPathVMax() { return 9.5; }

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

  struct AbsEncoderConstants {
    ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
        ::frc971::zeroing::AbsoluteEncoderZeroingEstimator>
        subsystem_params;
  };

  struct ArmJointConstants {
    ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants zeroing;
    double potentiometer_offset;
  };

  ArmJointConstants arm_proximal;
  ArmJointConstants arm_distal;
  ArmJointConstants roll_joint;

  AbsEncoderConstants wrist;

  bool wrist_flipped;
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
