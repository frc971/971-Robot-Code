#include "y2015/control_loops/claw/claw.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::Init();
  frc971::control_loops::Claw claw;
  claw.Run();
  ::aos::Cleanup();
  return 0;
}
