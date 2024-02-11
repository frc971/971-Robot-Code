#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/imu_fdcan/can_translator_lib.h"

DEFINE_string(channel, "/can", "The CAN channel to use");

using frc971::imu_fdcan::CANTranslator;

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());

  CANTranslator translator(&event_loop, FLAGS_channel);

  event_loop.Run();

  return 0;
}
