#include <stdio.h>

#include "aos/linux_code/init.h"
#include "frc971/actions/drivetrain_action.q.h"
#include "frc971/actions/drivetrain_actor.h"

using ::aos::time::Time;

int main(int /*argc*/, char * /*argv*/[]) {
  ::aos::Init();

  frc971::actions::DrivetrainActor drivetrain(
      &::frc971::actions::drivetrain_action);
  drivetrain.Run();

  ::aos::Cleanup();
  return 0;
}
