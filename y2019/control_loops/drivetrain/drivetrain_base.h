#ifndef Y2019_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
#define Y2019_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_

#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace y2019 {
namespace control_loops {
namespace drivetrain {

const ::frc971::control_loops::drivetrain::DrivetrainConfig<double>
    &GetDrivetrainConfig();

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2019

#endif  // Y2019_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
