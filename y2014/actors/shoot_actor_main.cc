#include <stdio.h>

#include "aos/events/shm-event-loop.h"
#include "aos/init.h"
#include "y2014/actors/shoot_action.q.h"
#include "y2014/actors/shoot_actor.h"

int main(int /*argc*/, char * /*argv*/[]) {
  ::aos::Init(-1);

  ::aos::ShmEventLoop event_loop;
  ::y2014::actors::ShootActor shoot(&event_loop);

  event_loop.Run();

  ::aos::Cleanup();
  return 0;
}
