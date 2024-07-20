#include "absl/flags/flag.h"

#include "aos/events/shm_event_loop.h"
#include "y2024_swerve/swerve_publisher_lib.h"

ABSL_FLAG(double, duration, 100.0, "Length of time in Ms to apply current for");
ABSL_FLAG(std::string, drivetrain_position, "swerve_drivetrain_output.json",
          "The path to the json drivetrain position to apply");
ABSL_FLAG(std::string, config, "aos_config.json",
          "The path to aos_config.json");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());

  std::unique_ptr<aos::ExitHandle> exit_handle = event_loop.MakeExitHandle();

  y2024_swerve::SwervePublisher publisher(
      &event_loop, exit_handle.get(), absl::GetFlag(FLAGS_drivetrain_position),
      absl::GetFlag(FLAGS_duration));

  event_loop.Run();
}
