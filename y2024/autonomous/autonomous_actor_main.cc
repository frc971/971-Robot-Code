#include <cstdio>

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/constants/constants_sender_lib.h"
#include "y2024/autonomous/autonomous_actor.h"

int main(int argc, char *argv[]) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  frc971::constants::WaitForConstants<y2024::Constants>(&config.message());

  ::aos::ShmEventLoop constant_fetcher_event_loop(&config.message());
  frc971::constants::ConstantsFetcher<y2024::Constants> constants_fetcher(
      &constant_fetcher_event_loop);

  const y2024::Constants *robot_constants = &constants_fetcher.constants();

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2024::autonomous::AutonomousActor autonomous(&event_loop, robot_constants);

  event_loop.Run();

  return 0;
}
