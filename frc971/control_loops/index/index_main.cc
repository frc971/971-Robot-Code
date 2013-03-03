#include "frc971/control_loops/index/index.h"

#include "aos/aos_core.h"

int main() {
  ::aos::Init();
  frc971::control_loops::IndexMotor indexer;
  indexer.Run();
  ::aos::Cleanup();
  return 0;
}
