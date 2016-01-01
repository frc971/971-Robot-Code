#include "y2012/control_loops/drivetrain/drivetrain.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::Init();
  ::y2012::control_loops::drivetrain::DrivetrainLoop drivetrain;
  drivetrain.Run();
  ::aos::Cleanup();
  return 0;
}
