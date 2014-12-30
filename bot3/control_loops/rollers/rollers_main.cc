#include "bot3/control_loops/rollers/rollers.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::Init();
  bot3::control_loops::RollersLoop rollers;
  rollers.Run();
  ::aos::Cleanup();
  return 0;
}
