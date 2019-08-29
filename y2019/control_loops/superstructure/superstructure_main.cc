#include "y2019/control_loops/superstructure/superstructure.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"

int main(int /*argc*/, char * /*argv*/ []) {
  ::aos::InitNRT(true);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2019::control_loops::superstructure::Superstructure superstructure(
      &event_loop);

  event_loop.Run();

  ::aos::Cleanup();
  return 0;
}
