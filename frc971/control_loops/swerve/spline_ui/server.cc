#include "seasocks/Server.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <array>
#include <cmath>
#include <memory>
#include <set>
#include <sstream>

#include "absl/flags/flag.h"
#include "google/protobuf/util/json_util.h"

#include "aos/containers/ring_buffer.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/seasocks/seasocks_logger.h"
#include "aos/time/time.h"
#include "internal/Embedded.h"
#include "seasocks/StringUtil.h"
#include "seasocks/WebSocket.h"

ABSL_FLAG(int, port, 1180, "Port number to serve on");

namespace frc971::control_loops::swerve::spline_ui {

namespace chrono = ::std::chrono;

class WebsocketHandler : public seasocks::WebSocket::Handler {
 public:
  WebsocketHandler();
  void onConnect(seasocks::WebSocket *connection) override;
  void onData(seasocks::WebSocket *connection, const char *data) override;
  void onDisconnect(seasocks::WebSocket *connection) override;

  void SendData(const std::string &data);

 private:
  std::set<seasocks::WebSocket *> connections_;
};

WebsocketHandler::WebsocketHandler() {}

void WebsocketHandler::onConnect(seasocks::WebSocket *connection) {
  connections_.insert(connection);
  AOS_LOG(INFO, "Connected: %s : %s\n", connection->getRequestUri().c_str(),
          seasocks::formatAddress(connection->getRemoteAddress()).c_str());
}

void WebsocketHandler::onData(seasocks::WebSocket *connection,
                              const char *data) {
  // TODO(Nathan): needs tests to check/modify file name
  AOS_LOG(INFO, "Got data: %s\n", data);
  connection->send("I got your data!");
  // parse websocket.send(filePath + '\n' + splineData);
  // write splineData to filePath
  std::string_view data_view(data);
  size_t newline = data_view.find('\n');
  if (newline == std::string_view::npos) {
    AOS_LOG(ERROR, "No newline found in data: %s\n", data);
    return;
  }
  std::string_view file_path = data_view.substr(0, newline);
  std::string_view spline_data = data_view.substr(newline + 1);
  aos::util::WriteStringToFileOrDie(
      absl::StrCat(getenv("BUILD_WORKSPACE_DIRECTORY"), file_path),
      spline_data);

  AOS_LOG(
      INFO, "Wrote data to file: %s\n",
      absl::StrCat(getenv("BUILD_WORKSPACE_DIRECTORY"), spline_data).c_str());
}

void WebsocketHandler::onDisconnect(seasocks::WebSocket *connection) {
  connections_.erase(connection);
  AOS_LOG(INFO, "Disconnected: %s : %s\n", connection->getRequestUri().c_str(),
          seasocks::formatAddress(connection->getRemoteAddress()).c_str());
}

void WebsocketHandler::SendData(const std::string &data) {
  for (seasocks::WebSocket *websocket : connections_) {
    websocket->send(data.c_str());
  }
}

}  // namespace frc971::control_loops::swerve::spline_ui

int main(int argc, char **argv) {
  // Make sure to reference this to force the linker to include it.
  findEmbeddedContent("");

  ::aos::InitGoogle(&argc, &argv);

  seasocks::Server server(::std::shared_ptr<seasocks::Logger>(
      new ::aos::seasocks::SeasocksLogger(seasocks::Logger::Level::Info)));

  auto websocket_handler = std::make_shared<
      frc971::control_loops::swerve::spline_ui::WebsocketHandler>();
  server.addWebSocketHandler("/ws", websocket_handler);

  server.serve("frc971/control_loops/swerve/spline_ui/www/static_files",
               absl::GetFlag(FLAGS_port));

  return 0;
}
