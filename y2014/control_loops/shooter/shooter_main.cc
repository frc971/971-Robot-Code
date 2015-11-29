#include "y2014/control_loops/shooter/shooter.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::Init();
  ::y2014::control_loops::ShooterMotor shooter;
  shooter.Run();
  ::aos::Cleanup();
  return 0;
}
