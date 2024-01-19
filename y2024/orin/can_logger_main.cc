#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/can_logger/can_logger.h"

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());

  frc971::can_logger::CanLogger cana_logger(&event_loop, "/can/cana", "cana");
  frc971::can_logger::CanLogger canb_logger(&event_loop, "/can/canb", "canb");
  frc971::can_logger::CanLogger canc_logger(&event_loop, "/can/canc", "canc");

  event_loop.Run();

  return 0;
}
