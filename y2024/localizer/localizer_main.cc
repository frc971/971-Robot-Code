#include "absl/flags/flag.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/constants/constants_sender_lib.h"
#include "y2024/control_loops/drivetrain/drivetrain_base.h"
#include "y2024/localizer/localizer.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  frc971::constants::WaitForConstants<y2024::Constants>(&config.message());

  aos::ShmEventLoop event_loop(&config.message());
  y2024::localizer::Localizer localizer(&event_loop);

  event_loop.Run();

  return 0;
}
