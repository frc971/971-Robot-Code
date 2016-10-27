#ifndef Y2016_BOT3_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
#define Y2016_BOT3_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_

#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace y2016_bot3 {
namespace constants {
static constexpr double drivetrain_max_speed = 5.0;

// The ratio from the encoder shaft to the drivetrain wheels.
static constexpr double kDrivetrainEncoderRatio = 1.0;

}  // namespace constants
namespace control_loops {
namespace drivetrain {
const ::frc971::control_loops::drivetrain::DrivetrainConfig &
GetDrivetrainConfig();

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2016_bot3

#endif  // Y2016_BOT3_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
