#include "y2015_bot3/control_loops/drivetrain/drivetrain.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::Init();
  ::y2015_bot3::control_loops::DrivetrainLoop drivetrain;
  drivetrain.Run();
  ::aos::Cleanup();
  return 0;
}
