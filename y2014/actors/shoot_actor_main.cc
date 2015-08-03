#include <stdio.h>

#include "aos/linux_code/init.h"
#include "y2014/actors/shoot_action.q.h"
#include "y2014/actors/shoot_actor.h"

using ::aos::time::Time;

int main(int /*argc*/, char * /*argv*/[]) {
  ::aos::Init();

  frc971::actors::ShootActor shoot(&::frc971::actors::shoot_action);
  shoot.Run();

  ::aos::Cleanup();
  return 0;
}

