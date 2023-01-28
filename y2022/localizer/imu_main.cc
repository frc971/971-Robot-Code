#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/realtime.h"
#include "frc971/imu_reader/imu.h"
#include "y2022/constants.h"

DEFINE_string(config, "aos_config.json", "Path to the config file to use.");

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  PCHECK(system("sudo chmod 644 /dev/adis16505") == 0)
      << ": Failed to set read permissions on IMU device.";

  aos::ShmEventLoop event_loop(&config.message());
  frc971::imu::Imu imu(&event_loop,
                       y2022::constants::Values::DrivetrainEncoderToMeters(1));

  event_loop.SetRuntimeRealtimePriority(30);

  event_loop.Run();

  return 0;
}
