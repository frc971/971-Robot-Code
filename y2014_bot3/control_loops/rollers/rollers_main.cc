#include "y2014_bot3/control_loops/rollers/rollers.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"

int main() {
  ::aos::InitNRT();

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2014_bot3::control_loops::rollers::Rollers rollers(&event_loop);

  event_loop.Run();

  return 0;
}
