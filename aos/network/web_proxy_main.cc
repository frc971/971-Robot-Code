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

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  // Make sure to reference this to force the linker to include it.
  findEmbeddedContent("");

  aos::InitNRT();

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  seasocks::Server server(std::shared_ptr<seasocks::Logger>(
      new aos::seasocks::SeasocksLogger(seasocks::Logger::Level::Info)));

  auto websocket_handler =
      std::make_shared<aos::web_proxy::WebsocketHandler>(&server, &event_loop);
  server.addWebSocketHandler("/ws", websocket_handler);

  std::thread data_thread{[&event_loop]() { event_loop.Run(); }};

  server.serve(FLAGS_data_dir.c_str(), 8080);
}
