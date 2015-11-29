#include "y2014/control_loops/claw/claw.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::Init();
  ::y2014::control_loops::ClawMotor claw;
  claw.Run();
  ::aos::Cleanup();
  return 0;
}
