#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "y2024/control_loops/superstructure/superstructure.h"

DEFINE_string(arm_trajectories, "arm_trajectories_generated.bfbs",
              "The path to the generated arm trajectories bfbs file.");

using y2024::control_loops::superstructure::Superstructure;

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());

  frc971::constants::WaitForConstants<y2024::Constants>(&config.message());

  std::shared_ptr<const y2024::constants::Values> values =
      std::make_shared<const y2024::constants::Values>(
          y2024::constants::MakeValues());
  Superstructure superstructure(&event_loop, values);

  event_loop.Run();

  return 0;
}
