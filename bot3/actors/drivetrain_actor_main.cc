#include <stdio.h>

#include "aos/linux_code/init.h"
#include "bot3/actors/drivetrain_action.q.h"
#include "bot3/actors/drivetrain_actor.h"

using ::aos::time::Time;

int main(int /*argc*/, char * /*argv*/[]) {
  ::aos::Init();

  frc971::actors::DrivetrainActor drivetrain(
      &::frc971::actors::drivetrain_action);
  drivetrain.Run();

  ::aos::Cleanup();
  return 0;
}
