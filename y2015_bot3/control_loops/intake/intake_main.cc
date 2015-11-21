#include "y2015_bot3/control_loops/intake/intake.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::Init();
  ::y2015_bot3::control_loops::Intake intake;
  intake.Run();
  ::aos::Cleanup();
  return 0;
}
