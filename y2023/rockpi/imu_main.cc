#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/realtime.h"
#include "frc971/imu_reader/imu.h"

DEFINE_string(config, "aos_config.json", "Path to the config file to use.");

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());
  // TODO(austin): Set the ratio...
  frc971::imu::Imu imu(&event_loop, 1.0);

  event_loop.SetRuntimeAffinity(aos::MakeCpusetFromCpus({0}));
  event_loop.SetRuntimeRealtimePriority(55);

  event_loop.Run();

  return 0;
}
