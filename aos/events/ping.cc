#include "absl/flags/flag.h"

#include "aos/configuration.h"
#include "aos/events/ping_lib.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"

ABSL_FLAG(std::string, config, "pingpong_config.json", "Path to the config.");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  aos::EventLoop::SetDefaultVersionString("ping_version");

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());

  aos::Ping ping(&event_loop);

  event_loop.Run();

  return 0;
}
