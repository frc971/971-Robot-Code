#include "aos/init.h"

#include "aos/events/shm-event-loop.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "y2019/control_loops/drivetrain/drivetrain_base.h"
#include "y2019/control_loops/drivetrain/event_loop_localizer.h"

using ::frc971::control_loops::drivetrain::DrivetrainLoop;

int main() {
  ::aos::Init();
  ::aos::ShmEventLoop event_loop;
  ::y2019::control_loops::drivetrain::EventLoopLocalizer localizer(
      ::y2019::control_loops::drivetrain::GetDrivetrainConfig(), &event_loop);
  DrivetrainLoop drivetrain(
      ::y2019::control_loops::drivetrain::GetDrivetrainConfig(), &event_loop,
      &localizer);
  drivetrain.Run();
  ::aos::Cleanup();
  return 0;
}
