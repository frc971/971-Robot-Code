#ifndef y2020_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
#define y2020_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_

#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace y2020::control_loops::drivetrain {

const ::frc971::control_loops::drivetrain::DrivetrainConfig<double> &
GetDrivetrainConfig();

}  // namespace y2020::control_loops::drivetrain

#endif  // y2020_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
