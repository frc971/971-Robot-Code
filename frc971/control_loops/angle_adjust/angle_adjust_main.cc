#include "frc971/control_loops/angle_adjust/angle_adjust.h"

#include "aos/atom_code/init.h"

int main() {
  ::aos::Init();
  ::frc971::control_loops::AngleAdjustMotor angle_adjust;
  angle_adjust.Run();
  ::aos::Cleanup();
  return 0;
}
