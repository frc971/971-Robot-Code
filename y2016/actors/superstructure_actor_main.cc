#include <stdio.h>

#include "aos/events/shm-event-loop.h"
#include "aos/init.h"
#include "y2016/actors/superstructure_action.q.h"
#include "y2016/actors/superstructure_actor.h"

int main(int /*argc*/, char* /*argv*/ []) {
  ::aos::Init(-1);

  ::aos::ShmEventLoop event_loop;
  ::y2016::actors::SuperstructureActor superstructure(&event_loop);
  event_loop.Run();

  ::aos::Cleanup();
  return 0;
}
