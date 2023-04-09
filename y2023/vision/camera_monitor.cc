#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "y2023/vision/camera_monitor_lib.h"

DEFINE_string(config, "aos_config.json", "Path to the config file to use.");

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  y2023::vision::CameraMonitor monitor(&event_loop);

  event_loop.Run();
}
