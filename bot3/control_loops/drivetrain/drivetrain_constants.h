#ifndef BOT3_CONTROL_LOOPS_DRIVETRAIN_CONSTANTS_H_
#define BOT3_CONTROL_LOOPS_DRIVETRAIN_CONSTANTS_H_

#include "bot3/shifter_hall_effect.h"

namespace bot3 {
namespace control_loops {

// TODO(danielp): Figure out the real values for these constants.
constexpr constants::ShifterHallEffect kBot3LeftDriveShifter =
    {170, 475, 1.2, 1.0};
constexpr constants::ShifterHallEffect kBot3RightDriveShifter =
    {177, 486, 1.2, 1.0};

constexpr double kBot3TurnWidth = 0.5;
constexpr double kBot3DrivetrainDoneDistance = 0.02;

constexpr double kBot3HighGearRatio = 18.0 / 60.0 * 18.0 / 50.0;
constexpr double kBot3LowGearRatio = 28.0 / 50.0 * 18.0 / 50.0;

}  // control_loops
}  // bot3

#endif
