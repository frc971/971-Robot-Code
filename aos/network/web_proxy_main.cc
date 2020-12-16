#include "aos/events/shm_event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/init.h"
#include "aos/network/web_proxy.h"
#include "aos/seasocks/seasocks_logger.h"
#include "gflags/gflags.h"

#include "internal/Embedded.h"
#include "seasocks/Server.h"
#include "seasocks/WebSocket.h"

DEFINE_string(config, "./config.json", "File path of aos configuration");
DEFINE_string(data_dir, "www", "Directory to serve data files from");
DEFINE_int32(buffer_size, 0, "-1 if infinite, in # of messages / channel.");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  // Make sure to reference this to force the linker to include it.
  findEmbeddedContent("");

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  aos::web_proxy::WebProxy web_proxy(&event_loop, FLAGS_buffer_size);
  web_proxy.SetDataPath(FLAGS_data_dir.c_str());

  event_loop.Run();
}
