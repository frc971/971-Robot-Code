#include <stdio.h>

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "y2014_bot3/actors/autonomous_actor.h"

int main(int /*argc*/, char * /*argv*/ []) {
  ::aos::Init(-1);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2014_bot3::actors::AutonomousActor autonomous(&event_loop);

  event_loop.Run();

  ::aos::Cleanup();
  return 0;
}
