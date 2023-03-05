#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "y2023/control_loops/superstructure/superstructure.h"

DEFINE_string(arm_trajectories, "arm_trajectories_generated.bfbs",
              "The path to the generated arm trajectories bfbs file.");

using y2023::control_loops::superstructure::Superstructure;
using y2023::control_loops::superstructure::arm::ArmTrajectories;

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());

  auto trajectories =
      y2023::control_loops::superstructure::Superstructure::GetArmTrajectories(
          FLAGS_arm_trajectories);

  std::shared_ptr<const y2023::constants::Values> values =
      std::make_shared<const y2023::constants::Values>(
          y2023::constants::MakeValues());
  Superstructure superstructure(&event_loop, values, trajectories);

  event_loop.Run();

  return 0;
}
