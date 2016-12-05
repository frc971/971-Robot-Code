#include "y2015/control_loops/fridge/fridge.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::Init();
  y2015::control_loops::fridge::Fridge fridge;
  fridge.Run();
  ::aos::Cleanup();
  return 0;
}
