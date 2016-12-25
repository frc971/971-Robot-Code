#include <stdio.h>

#include "aos/linux_code/init.h"
#include "y2014/actors/drivetrain_action.q.h"
#include "y2014/actors/drivetrain_actor.h"

int main(int /*argc*/, char * /*argv*/[]) {
  ::aos::Init(-1);

  ::y2014::actors::DrivetrainActor drivetrain(
      &::y2014::actors::drivetrain_action);
  drivetrain.Run();

  ::aos::Cleanup();
  return 0;
}
