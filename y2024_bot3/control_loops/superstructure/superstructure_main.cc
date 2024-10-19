#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "y2024_bot3/control_loops/superstructure/superstructure.h"

ABSL_FLAG(std::string, arm_trajectories, "arm_trajectories_generated.bfbs",
          "The path to the generated arm trajectories bfbs file.");

using y2024_bot3::control_loops::superstructure::Superstructure;

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());

  frc971::constants::WaitForConstants<y2024_bot3::Constants>(&config.message());

  Superstructure superstructure(&event_loop);

  event_loop.Run();

  return 0;
}
