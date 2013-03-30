#include "frc971/control_loops/drivetrain/drivetrain.h"

#include "aos/aos_core.h"

int main() {
  ::aos::Init();
  frc971::control_loops::DrivetrainLoop drivetrain;
  drivetrain.Run();
  ::aos::Cleanup();
  return 0;
}
