#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "y2022/control_loops/superstructure/superstructure.h"

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  std::shared_ptr<const y2022::constants::Values> values =
      std::make_shared<const y2022::constants::Values>(
          y2022::constants::MakeValues());
  ::y2022::control_loops::superstructure::Superstructure superstructure(
      &event_loop, values);

  event_loop.Run();

  return 0;
}
