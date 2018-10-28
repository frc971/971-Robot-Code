#include "y2017_bot3/control_loops/superstructure/superstructure.h"

#include "aos/init.h"

int main() {
  ::aos::Init();
  ::y2017_bot3::control_loops::superstructure::Superstructure superstructure;
  superstructure.Run();
  ::aos::Cleanup();
  return 0;
}
