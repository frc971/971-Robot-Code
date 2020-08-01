#include "y2014/control_loops/claw/claw.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"

int main() {
  ::aos::InitNRT();

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2014::control_loops::claw::ClawMotor claw(&event_loop);

  event_loop.Run();

  return 0;
}
