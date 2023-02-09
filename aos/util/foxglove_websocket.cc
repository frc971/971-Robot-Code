#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/util/foxglove_websocket_lib.h"
#include "gflags/gflags.h"

DEFINE_string(config, "/app/aos_config.json", "Path to the config.");
DEFINE_uint32(port, 8765, "Port to use for foxglove websocket server.");
DEFINE_string(mode, "flatbuffer", "json or flatbuffer serialization.");
DEFINE_bool(fetch_pinned_channels, true,
            "Set this to allow foxglove_websocket to make fetchers on channels "
            "with a read_method of PIN (see aos/configuration.fbs; PIN is an "
            "enum value). Having this enabled will cause foxglove to  consume "
            "extra shared memory resources.");

int main(int argc, char *argv[]) {
  gflags::SetUsageMessage(
      "Runs a websocket server that a foxglove instance can connect to in "
      "order to view live data on a device.\n\n"
      "Typical Usage: foxglove_websocket [--port 8765]\n"
      "If the default port is not exposed directly, you can port-forward with "
      "SSH by doing\n"
      "$ ssh -L 8765:localhost:8765 ssh_target\n\n"
      "When accessing this in foxglove:\n"
      "1) Open a data source (this window may be open by default).\n"
      "2) Select \"Open Connection\"\n"
      "3) Select \"Foxglove WebSocket\" (do NOT select the rosbridge option)\n"
      "4) Fill out the URL for the machine. If port forwarding, the default\n"
      "   ws://localhost:8765 should work.\n\n"
      "Note that this does not start up a foxglove instance itself. You must "
      "either have one locally on your laptop, or go to "
      "https://studio.foxglove.dev, or use another application to serve the "
      "foxglove HTML pages.\n"
      "If you want to use the studio.foxglove.dev page to view data (which "
      "won't send any of your data to foxglove.dev--it's just needed to load "
      "the HTML files), you can also go directly to:\n"
      "https://studio.foxglove.dev/?ds=foxglove-websocket&ds.url=ws://"
      "localhost:8765\n"
      "where localhost:8765 must be updated if you aren't port-forwarding "
      "and/or are using a different port number. Similarly, if you are serving "
      "the static foxglove files locally, you can update the "
      "studio.foxglove.dev to point at your local webserver.\n");
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  aos::FoxgloveWebsocketServer server(
      &event_loop, FLAGS_port,
      FLAGS_mode == "flatbuffer"
          ? aos::FoxgloveWebsocketServer::Serialization::kFlatbuffer
          : aos::FoxgloveWebsocketServer::Serialization::kJson,
      FLAGS_fetch_pinned_channels
          ? aos::FoxgloveWebsocketServer::FetchPinnedChannels::kYes
          : aos::FoxgloveWebsocketServer::FetchPinnedChannels::kNo);

  event_loop.Run();
}
