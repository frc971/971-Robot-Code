#include "y2019/control_loops/superstructure/superstructure.h"

#include "aos/init.h"

int main() {
  ::aos::Init();
  ::y2019::control_loops::superstructure::Superstructure superstructure;
  superstructure.Run();
  ::aos::Cleanup();
  return 0;
}
