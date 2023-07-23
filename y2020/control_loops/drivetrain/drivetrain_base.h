#ifndef y2020_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
#define y2020_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_

#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace y2020 {
namespace control_loops {
namespace drivetrain {

const ::frc971::control_loops::drivetrain::DrivetrainConfig<double> &
GetDrivetrainConfig();

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2020

#endif  // y2020_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
