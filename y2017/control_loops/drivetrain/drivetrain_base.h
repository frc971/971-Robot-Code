#ifndef Y2017_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
#define Y2017_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_

#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace y2017 {
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
}  // namespace y2017

#endif  // Y2017_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
