#ifndef Y2023_BOT4_CONSTANTS_H
#define Y2023_BOT4_CONSTANTS_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <cstdint>

#include "frc971/constants.h"

namespace y2023_bot4 {
namespace constants {
struct Values {
  static const int kZeroingSampleSize = 200;

  static const int kDrivetrainWriterPriority = 35;
  static const int kDrivetrainTxPriority = 36;
  static const int kDrivetrainRxPriority = 36;

  // TODO (maxwell): Make this the real value;
  static constexpr double kDrivetrainCyclesPerRevolution() { return 512.0; }
  static constexpr double kDrivetrainEncoderRatio() { return 1.0; }

  static constexpr double kDrivetrainStatorCurrentLimit() { return 35.0; }
  static constexpr double kDrivetrainSupplyCurrentLimit() { return 60.0; }

  // TODO (maxwell): Make this the real value
  static constexpr double kFollowerWheelCountsPerRevolution() { return 512.0; }
  static constexpr double kFollowerWheelEncoderRatio() { return 1.0; }
  static constexpr double kFollowerWheelRadius() { return 3.25 / 2 * 0.0254; }
  static constexpr double kDrivetrainEncoderCountsPerRevolution() {
    return 2048.0;
  }

  static constexpr double kMaxDrivetrainEncoderPulsesPerSecond() {
    return 1200000;
  }

  frc971::constants::ContinuousAbsoluteEncoderZeroingConstants
      front_left_zeroing_constants,
      front_right_zeroing_constants, back_left_zeroing_constants,
      back_right_zeroing_constants;
};
// Creates and returns a Values instance for the constants.
// Should be called before realtime because this allocates memory.
// Only the first call to either of these will be used.
Values MakeValues(uint16_t team);

// Calls MakeValues with aos::network::GetTeamNumber()
Values MakeValues();
}  // namespace constants
}  // namespace y2023_bot4

#endif  // Y2023_BOT4_CONSTANTS_H
