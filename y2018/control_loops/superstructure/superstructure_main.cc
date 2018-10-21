#include "y2018/control_loops/superstructure/superstructure.h"

#include "aos/init.h"

int main() {
  ::aos::InitNRT(true);
  ::y2018::control_loops::superstructure::Superstructure superstructure;
  ::aos::GoRT();
  superstructure.Run();
  ::aos::Cleanup();
  return 0;
}
