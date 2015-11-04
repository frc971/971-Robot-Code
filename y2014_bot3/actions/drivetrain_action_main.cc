#include <stdio.h>

#include "aos/linux_code/init.h"
#include "aos/common/logging/logging.h"
#include "bot3/actions/drivetrain_action.h"
#include "frc971/actions/drivetrain_action.q.h"

using ::aos::time::Time;

int main(int /*argc*/, char * /*argv*/[]) {
  ::aos::Init();

  bot3::actions::DrivetrainAction drivetrain(&::frc971::actions::drivetrain_action);
  drivetrain.Run();

  ::aos::Cleanup();
  return 0;
}

