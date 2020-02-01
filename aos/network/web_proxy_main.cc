#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/network/web_proxy.h"
#include "aos/seasocks/seasocks_logger.h"
#include "gflags/gflags.h"

#include "internal/Embedded.h"
#include "seasocks/Server.h"
#include "seasocks/WebSocket.h"

DEFINE_string(config, "./config.json", "File path of aos configuration");
DEFINE_string(data_dir, "www", "Directory to serve data files from");

void RunDataThread(
    std::vector<std::unique_ptr<aos::web_proxy::Subscriber>> *subscribers,
    const aos::FlatbufferDetachedBuffer<aos::Configuration> &config) {
  aos::ShmEventLoop event_loop(&config.message());

  // TODO(alex): skip fetchers on the wrong node.
  for (uint i = 0; i < config.message().channels()->size(); ++i) {
    auto channel = config.message().channels()->Get(i);
    auto fetcher = event_loop.MakeRawFetcher(channel);
    subscribers->emplace_back(
        std::make_unique<aos::web_proxy::Subscriber>(std::move(fetcher), i));
  }

  flatbuffers::FlatBufferBuilder fbb(1024);

  auto timer = event_loop.AddTimer([&]() {
    for (auto &subscriber : *subscribers) {
      subscriber->RunIteration();
    }
  });

  event_loop.OnRun([&]() {
    timer->Setup(event_loop.monotonic_now(), std::chrono::milliseconds(100));
  });

  event_loop.Run();
}

int main(int argc, char **argv) {
  // Make sure to reference this to force the linker to include it.
  aos::InitGoogle(&argc, &argv);
  findEmbeddedContent("");

  aos::InitNRT();

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  std::vector<std::unique_ptr<aos::web_proxy::Subscriber>> subscribers;

  std::thread data_thread{
      [&subscribers, &config]() { RunDataThread(&subscribers, config); }};

  seasocks::Server server(std::shared_ptr<seasocks::Logger>(
      new aos::seasocks::SeasocksLogger(seasocks::Logger::Level::Info)));

  auto websocket_handler = std::make_shared<aos::web_proxy::WebsocketHandler>(
      &server, subscribers, config);
  server.addWebSocketHandler("/ws", websocket_handler);

  server.serve(FLAGS_data_dir.c_str(), 8080);
}
