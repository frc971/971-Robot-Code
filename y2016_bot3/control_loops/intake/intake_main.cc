#include "y2016_bot3/control_loops/intake/intake.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::Init();
  ::y2016_bot3::control_loops::intake::Intake intake;
  intake.Run();
  ::aos::Cleanup();
  return 0;
}
