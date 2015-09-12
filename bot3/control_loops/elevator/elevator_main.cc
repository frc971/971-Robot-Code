#include "bot3/control_loops/elevator/elevator.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::Init();
  ::bot3::control_loops::Elevator elevator;
  elevator.Run();
  ::aos::Cleanup();
  return 0;
}
