#include <stdio.h>

#include "aos/logging/logging.h"
#include "aos/init.h"
#include "bot3/actions/drivetrain_action.h"
#include "frc971/actions/drivetrain_action.q.h"

int main(int /*argc*/, char * /*argv*/[]) {
  ::aos::Init();

  bot3::actions::DrivetrainAction drivetrain(&::frc971::actions::drivetrain_action);
  drivetrain.Run();

  ::aos::Cleanup();
  return 0;
}

