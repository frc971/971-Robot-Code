#ifndef BOT3_CONTROL_LOOPS_DRIVETRAIN_CONSTANTS_H_
#define BOT3_CONTROL_LOOPS_DRIVETRAIN_CONSTANTS_H_

#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/shifter_hall_effect.h"

namespace bot3 {
namespace control_loops {

// TODO(danielp): Figure out the real values for these constants.
constexpr ::frc971::constants::ShifterHallEffect kBot3LeftDriveShifter =
    {555, 657, 660, 560, 0.2, 0.7};
constexpr ::frc971::constants::ShifterHallEffect kBot3RightDriveShifter =
    {555, 660, 644, 552, 0.2, 0.7};

constexpr double kBot3TurnWidth = 0.5;
constexpr double kBot3DrivetrainDoneDistance = 0.02;

constexpr double kBot3HighGearRatio = 18.0 / 60.0 * 18.0 / 50.0;
constexpr double kBot3LowGearRatio = 28.0 / 50.0 * 18.0 / 50.0;

}  // control_loops
}  // bot3

#endif
