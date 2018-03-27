#ifndef Y2014_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
#define Y2014_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_

#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace y2014 {
namespace control_loops {

const ::frc971::control_loops::drivetrain::DrivetrainConfig<double>
    &GetDrivetrainConfig();

}  // namespace control_loops
}  // namespace y2014

#endif  // Y2014_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
