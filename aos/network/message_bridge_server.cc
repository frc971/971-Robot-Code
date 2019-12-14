#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/network/message_bridge_server_lib.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(config, "multinode_pingpong_config.json", "Path to the config.");

namespace aos {
namespace message_bridge {

int Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  MessageBridgeServer app(&event_loop);

  // TODO(austin): Track which messages didn't make it in time and need to be
  // logged locally and forwarded.

  event_loop.Run();

  return EXIT_SUCCESS;
}

}  // namespace message_bridge
}  // namespace aos

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  return aos::message_bridge::Main();
}
