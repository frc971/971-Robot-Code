#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/constants/constants_sender_lib.h"
#include "y2023/control_loops/drivetrain/drivetrain_base.h"
#include "y2023/localizer/localizer.h"

DEFINE_string(config, "aos_config.json", "Path to the config file to use.");

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  frc971::constants::WaitForConstants<y2023::Constants>(&config.message());

  aos::ShmEventLoop event_loop(&config.message());
  y2023::localizer::Localizer localizer(
      &event_loop, ::y2023::control_loops::drivetrain::GetDrivetrainConfig());

  event_loop.Run();

  return 0;
}
