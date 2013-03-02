#include "frc971/control_loops/wrist.h"

#include "aos/aos_core.h"

int main() {
  ::aos::Init();
  frc971::control_loops::WristMotor wrist;
  wrist.Run();
  ::aos::Cleanup();
  return 0;
}
