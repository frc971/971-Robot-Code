#include "frc971/control_loops/wrist/wrists.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::Init();
  frc971::control_loops::WristsLoop wrists;
  wrists.Run();
  ::aos::Cleanup();
  return 0;
}
