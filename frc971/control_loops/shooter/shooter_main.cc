#include "frc971/control_loops/shooter/shooter.h"

#include "aos/atom_code/init.h"

int main() {
  ::aos::Init();
  frc971::control_loops::ShooterMotor shooter;
  shooter.Run();
  ::aos::Cleanup();
  return 0;
}
