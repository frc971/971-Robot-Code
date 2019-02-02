#include "aos/init.h"

#include "aos/events/shm-event-loop.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "y2014/control_loops/drivetrain/drivetrain_base.h"

using ::frc971::control_loops::drivetrain::DrivetrainLoop;

int main() {
  ::aos::Init();
  ::aos::ShmEventLoop event_loop;
  DrivetrainLoop drivetrain(::y2014::control_loops::GetDrivetrainConfig(),
                            &event_loop);
  drivetrain.Run();
  ::aos::Cleanup();
  return 0;
}
