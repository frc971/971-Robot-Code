#include <stdio.h>

#include "aos/linux_code/init.h"
#include "y2014/actors/shoot_action.q.h"
#include "y2014/actors/shoot_actor.h"

int main(int /*argc*/, char * /*argv*/[]) {
  ::aos::Init(-1);

  ::y2014::actors::ShootActor shoot(&::y2014::actors::shoot_action);
  shoot.Run();

  ::aos::Cleanup();
  return 0;
}

