#include "absl/flags/flag.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "y2022/control_loops/drivetrain/drivetrain_base.h"
#include "y2022/localizer/localizer.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());
  frc971::controls::EventLoopLocalizer localizer(
      &event_loop, ::y2022::control_loops::drivetrain::GetDrivetrainConfig());

  event_loop.Run();

  return 0;
}
