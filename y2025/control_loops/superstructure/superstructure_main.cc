#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "y2025/control_loops/superstructure/superstructure.h"

using y2025::control_loops::superstructure::Superstructure;

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());

  frc971::constants::WaitForConstants<y2025::Constants>(&config.message());

  Superstructure superstructure(&event_loop);

  event_loop.Run();

  return 0;
}
