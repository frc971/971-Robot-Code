#ifndef Y2016_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
#define Y2016_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_

#include "frc971/control_loops/drivetrain/drivetrain_config.h"

using ::frc971::control_loops::drivetrain::DrivetrainConfig;

namespace y2016 {
namespace control_loops {

const DrivetrainConfig &GetDrivetrainConfig();

}  // namespace control_loops
}  // namespace y2016

#endif  // Y2016_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
