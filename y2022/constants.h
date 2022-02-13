#ifndef Y2022_CONSTANTS_H_
#define Y2022_CONSTANTS_H_

#include <array>
#include <cmath>
#include <cstdint>

#include "frc971/constants.h"
#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "y2022/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2022/control_loops/superstructure/intake/intake_plant.h"

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
           kDrivetrainEncoderRatio() *
           control_loops::drivetrain::kWheelRadius;
  }

  static constexpr double kRollerSupplyCurrentLimit() { return 30.0; }
  static constexpr double kRollerStatorCurrentLimit() { return 40.0; }

  ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
      ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator>
      intake;

  // TODO (Yash): Constants need to be tuned
  static constexpr ::frc971::constants::Range kIntakeRange() {
    return ::frc971::constants::Range{
        .lower_hard = -0.5,         // Back Hard
        .upper_hard = 2.85 + 0.05,  // Front Hard
        .lower = -0.300,            // Back Soft
        .upper = 2.725              // Front Soft
    };
  }
  // Climber
  static constexpr double kClimberSupplyCurrentLimit() { return 60.0; }

  // Intake
  // two encoders with same gear ratio for intake
  static constexpr double kIntakeEncoderCountsPerRevolution() { return 512.0; }

  static constexpr double kIntakeEncoderRatio() {
    return ((16.0 / 60.0) * (18.0 / 62.0));
  }

  // TODO(Milo): Also need to add specific PPR (Pulse per revolution)
};

// Creates (once) a Values instance for ::aos::network::GetTeamNumber(). Should
// be called before realtime because this allocates memory.
void InitValues();

// Returns a reference to the Values instance for
// ::aos::network::GetTeamNumber(). Values must be initialized through
// InitValues() before calling this.
const Values &GetValues();

}  // namespace constants
}  // namespace y2022

#endif  // Y2022_CONSTANTS_H_
