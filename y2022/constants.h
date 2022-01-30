#ifndef Y2022_CONSTANTS_H_
#define Y2022_CONSTANTS_H_

#include <array>
#include <cmath>
#include <cstdint>

#include "frc971/constants.h"
#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "y2022/control_loops/drivetrain/drivetrain_dog_motor_plant.h"

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
  static constexpr double kRollerSupplyCurrentLimit() { return 30.0; }
  static constexpr double kRollerStatorCurrentLimit() { return 40.0; }

  // Climber
  static constexpr double kClimberSupplyCurrentLimit() { return 60.0; }
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
