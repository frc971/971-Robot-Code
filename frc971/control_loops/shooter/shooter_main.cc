#include "frc971/control_loops/shooter/shooter.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::Init();
  frc971::control_loops::ShooterLoop shooter;
  shooter.Run();
  ::aos::Cleanup();
  return 0;
}
