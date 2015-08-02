#include "y2015/control_loops/fridge/fridge.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::Init();
  frc971::control_loops::Fridge fridge;
  fridge.Run();
  ::aos::Cleanup();
  return 0;
}
