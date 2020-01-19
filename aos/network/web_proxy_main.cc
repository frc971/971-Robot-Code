#include "aos/init.h"
#include "aos/seasocks/seasocks_logger.h"
#include "aos/network/web_proxy.h"

#include "internal/Embedded.h"
#include "seasocks/Server.h"
#include "seasocks/WebSocket.h"

int main() {
  // Make sure to reference this to force the linker to include it.
  findEmbeddedContent("");

  aos::InitNRT();

  seasocks::Server server(::std::shared_ptr<seasocks::Logger>(
      new ::aos::seasocks::SeasocksLogger(seasocks::Logger::Level::Info)));

  auto websocket_handler =
      std::make_shared<aos::web_proxy::WebsocketHandler>(&server);
  server.addWebSocketHandler("/ws", websocket_handler);

  server.serve("aos/network/www", 8080);
}
