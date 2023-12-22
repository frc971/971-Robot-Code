#ifndef Y2023_BOT3_CONSTANTS_H_
#define Y2023_BOT3_CONSTANTS_H_

#include <array>
#include <cmath>
#include <cstdint>

#include "frc971/constants.h"
#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "frc971/zeroing/pot_and_absolute_encoder.h"
#include "y2023_bot3/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2023_bot3/control_loops/superstructure/pivot_joint/pivot_joint_plant.h"
namespace y2023_bot3 {
namespace constants {

constexpr uint16_t kThirdRobotTeamNumber = 9984;

struct Values {
  static const int kZeroingSampleSize = 200;

  static const int kSuperstructureCANWriterPriority = 35;
  static const int kDrivetrainWriterPriority = 35;
  static const int kDrivetrainTxPriority = 36;
  static const int kDrivetrainRxPriority = 36;

  // TODO(max): Change these constants based on 3rd drivetrain CAD
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

  static constexpr double kRollerSupplyCurrentLimit() { return 30.0; }
  static constexpr double kRollerStatorCurrentLimit() { return 100.0; }

  static constexpr double kPivotSupplyCurrentLimit() { return 40.0; }
  static constexpr double kPivotStatorCurrentLimit() { return 200.0; }

  // timeout to ensure code doesn't get stuck after releasing the "intake"
  // button
  static constexpr std::chrono::milliseconds kExtraIntakingTime() {
    return std::chrono::milliseconds{100};
  }

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

  // Pivot Joint
  static constexpr double kPivotJointEncoderCountsPerRevolution() {
    return 4096.0;
  }

  static constexpr double kPivotJointEncoderRatio() {
    return (24.0 / 64.0) * (15.0 / 60.0);
  }

  static constexpr double kPivotJointPotRatio() {
    return (24.0 / 64.0) * (15.0 / 60.0);
  }

  static constexpr double kPivotJointPotRadiansPerVolt() {
    return kPivotJointPotRatio() * (10.0 /*turns*/ / 5.0 /*volts*/) *
           (2 * M_PI /*radians*/);
  }

  static constexpr double kMaxPivotJointEncoderPulsesPerSecond() {
    return control_loops::superstructure::pivot_joint::kFreeSpeed /
           (2.0 * M_PI) *
           control_loops::superstructure::pivot_joint::kOutputRatio /
           kPivotJointEncoderRatio() * kPivotJointEncoderCountsPerRevolution();
  }

  static constexpr ::frc971::constants::Range kPivotJointRange() {
    return ::frc971::constants::Range{
        .lower_hard = -1.78879503977269,  // Back Hard
        .upper_hard = 1.76302285774785,   // Front Hard
        .lower = -1.77156498873494,       // Back Soft
        .upper = 1.76555657862879,        // Front Soft
    };
  }

  struct PotAndAbsEncoderConstants {
    ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
        ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator>
        subsystem_params;
    double potentiometer_offset;
  };

  PotAndAbsEncoderConstants pivot_joint;

  bool pivot_joint_flipped;
};

// Creates and returns a Values instance for the constants.
// Should be called before realtime because this allocates memory.
// Only the first call to either of these will be used.
Values MakeValues(uint16_t team);

// Calls MakeValues with aos::network::GetTeamNumber()
Values MakeValues();

}  // namespace constants
}  // namespace y2023_bot3

#endif  // Y2023_BOT3_CONSTANTS_H_
