#include "gflags/gflags.h"
#include "glog/logging.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/dynamic_logging.h"
#include "aos/network/message_bridge_server_lib.h"
#include "aos/network/sctp_lib.h"
#include "aos/sha256.h"

DEFINE_string(config, "aos_config.json", "Path to the config.");
DEFINE_int32(rt_priority, -1, "If > 0, run as this RT priority");

#if HAS_SCTP_AUTH
DEFINE_string(sctp_auth_key_file, "",
              "When set, use the provided key for SCTP authentication as "
              "defined in RFC 4895");
#endif

namespace aos {
namespace message_bridge {

using ::aos::util::ReadFileToVecOrDie;

int Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());
  if (FLAGS_rt_priority > 0) {
    event_loop.SetRuntimeRealtimePriority(FLAGS_rt_priority);
  }

  std::vector<uint8_t> auth_key;
#if HAS_SCTP_AUTH
  if (!FLAGS_sctp_auth_key_file.empty()) {
    auth_key = ReadFileToVecOrDie(FLAGS_sctp_auth_key_file);
  }
#endif
  MessageBridgeServer app(&event_loop, Sha256(config.span()),
                          std::move(auth_key));

  logging::DynamicLogging dynamic_logging(&event_loop);

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
