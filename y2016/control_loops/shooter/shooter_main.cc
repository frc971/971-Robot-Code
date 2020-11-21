#include "y2016/control_loops/shooter/shooter.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2016::control_loops::shooter::Shooter shooter(&event_loop);

  event_loop.Run();

  return 0;
}
