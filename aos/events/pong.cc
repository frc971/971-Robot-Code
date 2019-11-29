#include "aos/configuration.h"
#include "aos/events/ping_generated.h"
#include "aos/events/pong_generated.h"
#include "aos/events/pong_lib.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "glog/logging.h"

int main(int argc, char **argv) {
  FLAGS_logtostderr = true;
  google::InitGoogleLogging(argv[0]);
  ::gflags::ParseCommandLineFlags(&argc, &argv, true);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos/events/pingpong_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());

  aos::Pong ping(&event_loop);

  event_loop.Run();

  ::aos::Cleanup();
  return 0;
}
