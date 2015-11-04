#ifndef BOT3_CONTROL_LOOPS_DRIVETRAIN_CONSTANTS_H_
#define BOT3_CONTROL_LOOPS_DRIVETRAIN_CONSTANTS_H_

#include "bot3/shifter_hall_effect.h"

namespace bot3 {
namespace control_loops {

constexpr constants::ShifterHallEffect kBot3LeftDriveShifter =
    {426, 171, 0.6, 0.47};
constexpr constants::ShifterHallEffect kBot3RightDriveShifter =
    {424, 172, 0.62, 0.55};

constexpr double kBot3TurnWidth = 0.5;
constexpr double kBot3DrivetrainDoneDistance = 0.02;

constexpr double kBot3LowGearRatio = 14.0 / 60.0 * 17.0 / 50.0;
constexpr double kBot3HighGearRatio = 30.0 / 44.0 * 17.0 / 50.0;

// If this is true, we don't use the analog hall effects for shifting.
constexpr bool kBot3SimpleShifting = true;

}  // control_loops
}  // bot3

#endif
