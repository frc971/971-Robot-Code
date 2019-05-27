#include <stdio.h>

#include "aos/events/shm-event-loop.h"
#include "aos/init.h"
#include "y2016/actors/autonomous_actor.h"

int main(int /*argc*/, char * /*argv*/ []) {
  ::aos::Init(-1);

  ::aos::ShmEventLoop event_loop;
  ::y2016::actors::AutonomousActor autonomous(&event_loop);
  event_loop.Run();

  ::aos::Cleanup();
  return 0;
}
