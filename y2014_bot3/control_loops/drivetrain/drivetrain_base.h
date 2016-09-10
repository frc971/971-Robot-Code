#ifndef Y2014_BOT3_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
#define Y2014_BOT3_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_

#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace y2014_bot3 {
namespace control_loops {
namespace drivetrain {

const double kDrivetrainEncoderRatio =
    (17.0 / 50.0) /*output reduction*/ * (24.0 / 64.0) /*encoder gears*/;

const ::frc971::control_loops::drivetrain::DrivetrainConfig &
GetDrivetrainConfig();

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2014_bot3

#endif  // Y2014_BOT3_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
