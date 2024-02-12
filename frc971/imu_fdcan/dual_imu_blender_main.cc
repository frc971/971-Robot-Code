#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/imu_fdcan/dual_imu_blender_lib.h"

using frc971::imu_fdcan::DualImuBlender;

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());

  DualImuBlender blender(&event_loop);

  event_loop.Run();

  return 0;
}
