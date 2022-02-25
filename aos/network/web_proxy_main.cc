#include "aos/events/shm_event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/init.h"
#include "aos/network/web_proxy.h"
#include "gflags/gflags.h"

DEFINE_string(config, "./aos_config.json", "File path of aos configuration");
DEFINE_string(data_dir, "www", "Directory to serve data files from");
DEFINE_int32(buffer_size, 1000000,
             "-1 if infinite, in bytes / channel. If there are no active "
             "connections, will not store anything.");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  aos::web_proxy::WebProxy web_proxy(
      &event_loop, aos::web_proxy::StoreHistory::kNo, FLAGS_buffer_size);
  web_proxy.SetDataPath(FLAGS_data_dir.c_str());

  event_loop.Run();
}
