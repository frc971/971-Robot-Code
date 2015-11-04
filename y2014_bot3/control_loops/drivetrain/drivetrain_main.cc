#include "y2014_bot3/control_loops/drivetrain/drivetrain.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::Init();
  y2014_bot3::control_loops::DrivetrainLoop drivetrain;
  drivetrain.Run();
  ::aos::Cleanup();
  return 0;
}
