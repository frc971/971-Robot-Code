#include "frc971/control_loops/wrist/wrist.h"

#include "aos/atom_code/init.h"

int main() {
  ::aos::Init();
  frc971::control_loops::WristMotor wrist;
  wrist.Run();
  ::aos::Cleanup();
  return 0;
}
