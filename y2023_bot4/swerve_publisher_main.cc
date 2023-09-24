#include "aos/events/shm_event_loop.h"
#include "y2023_bot4/swerve_publisher_lib.h"

DEFINE_double(duration, 100.0, "Length of time in Ms to apply current for");
DEFINE_string(drivetrain_position, "swerve_drivetrain_output.json",
              "The path to the json drivetrain position to apply");
DEFINE_string(config, "aos_config.json", "The path to aos_config.json");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  std::unique_ptr<aos::ExitHandle> exit_handle = event_loop.MakeExitHandle();

  y2023_bot4::SwervePublisher publisher(&event_loop, exit_handle.get(),
                                        FLAGS_drivetrain_position,
                                        FLAGS_duration);

  event_loop.Run();
}
