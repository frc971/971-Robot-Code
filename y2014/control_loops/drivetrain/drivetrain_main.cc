#include "aos/linux_code/init.h"

#include "y2014/control_loops/drivetrain/drivetrain_base.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"

using ::frc971::control_loops::drivetrain::DrivetrainLoop;

int main() {
  ::aos::Init();
  DrivetrainLoop drivetrain(::y2014::control_loops::GetDrivetrainConfig());
  drivetrain.Run();
  ::aos::Cleanup();
  return 0;
}
