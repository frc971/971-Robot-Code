#include "y2016/control_loops/shooter/shooter.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::Init();
  ::y2016::control_loops::shooter::Shooter shooter;
  shooter.Run();
  ::aos::Cleanup();
  return 0;
}
