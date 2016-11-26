#ifndef Y2015_BOT3_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
#define Y2015_BOT3_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_

#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace y2015_bot3 {
namespace control_loops {
namespace drivetrain {

// Constants
// TODO(comran): Get actual constants.
constexpr double kDrivetrainTurnWidth = 0.63;
constexpr double kDrivetrainEncoderRatio = 18.0 / 44.0;
constexpr double kDrivetrainHighGearRatio =
    kDrivetrainEncoderRatio * 18.0 / 60.0;
constexpr double kDrivetrainLowGearRatio = kDrivetrainHighGearRatio;
const bool kDrivetrainClutchTransmission = false;
const ::frc971::constants::ShifterHallEffect kDrivetrainRightShifter{
    555, 657, 660, 560, 0.2, 0.7};
const ::frc971::constants::ShifterHallEffect kDrivetrainLeftShifter{
    555, 660, 644, 552, 0.2, 0.7};
// End constants

const ::frc971::control_loops::drivetrain::DrivetrainConfig &
GetDrivetrainConfig();

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2015_bot3

#endif  // Y2015_BOT3_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
