#ifndef Y2014_BOT3_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
#define Y2014_BOT3_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_

#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace y2014_bot3::control_loops::drivetrain {

const double kDrivetrainEncoderRatio =
    (17.0 / 50.0) /*output reduction*/ * (64.0 / 24.0) /*encoder gears*/;

const ::frc971::control_loops::drivetrain::DrivetrainConfig<double> &
GetDrivetrainConfig();

}  // namespace y2014_bot3::control_loops::drivetrain

#endif  // Y2014_BOT3_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
