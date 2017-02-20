#include "y2017/control_loops/superstructure/superstructure.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::Init();
  ::y2017::control_loops::superstructure::Superstructure superstructure;
  superstructure.Run();
  ::aos::Cleanup();
  return 0;
}
