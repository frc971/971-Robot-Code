#ifndef Y2024_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
#define Y2024_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_

#include "aos/events/event_loop.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace y2024::control_loops::drivetrain {

const ::frc971::control_loops::drivetrain::DrivetrainConfig<double>
GetDrivetrainConfig(aos::EventLoop *event_loop);

}  // namespace y2024::control_loops::drivetrain

#endif  // Y2024_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
