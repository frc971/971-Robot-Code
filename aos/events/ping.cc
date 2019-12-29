#include "aos/configuration.h"
#include "aos/events/ping_lib.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(config, "aos/events/pingpong_config.json", "Path to the config.");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  aos::Ping ping(&event_loop);

  event_loop.Run();

  aos::Cleanup();
  return 0;
}
