#include "y2014_bot3/control_loops/rollers/rollers.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::Init();
  ::y2014_bot3::control_loops::Rollers rollers;
  rollers.Run();
  ::aos::Cleanup();
  return 0;
}
