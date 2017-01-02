#ifndef Y2016_BOT4_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
#define Y2016_BOT4_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_

#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace y2016_bot4 {
namespace constants {
// The ratio from the encoder shaft to the drivetrain wheels.
static constexpr double kDrivetrainEncoderRatio = 1.0;

}  // namespace constants
namespace control_loops {
namespace drivetrain {
const ::frc971::control_loops::drivetrain::DrivetrainConfig &
GetDrivetrainConfig();

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2016_bot4

#endif  // Y2016_BOT4_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
