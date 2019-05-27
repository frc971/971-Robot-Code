#include <stdio.h>

#include "aos/events/shm-event-loop.h"
#include "aos/init.h"
#include "frc971/autonomous/auto.q.h"
#include "y2017/actors/autonomous_actor.h"

int main(int /*argc*/, char * /*argv*/ []) {
  ::aos::Init(-1);

  ::aos::ShmEventLoop event_loop;
  ::y2017::actors::AutonomousActor autonomous(&event_loop);
  event_loop.Run();

  ::aos::Cleanup();
  return 0;
}
