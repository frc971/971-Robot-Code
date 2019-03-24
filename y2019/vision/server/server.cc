#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <array>
#include <memory>
#include <set>
#include <sstream>

#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/time/time.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "internal/Embedded.h"
#include "seasocks/PrintfLogger.h"
#include "seasocks/Server.h"
#include "seasocks/StringUtil.h"
#include "seasocks/WebSocket.h"
#include "y2019/control_loops/drivetrain/camera.q.h"

namespace y2019 {
namespace vision {

class WebsocketHandler : public seasocks::WebSocket::Handler {
 public:
  WebsocketHandler();
  void onConnect(seasocks::WebSocket* connection) override;
  void onData(seasocks::WebSocket* connection, const char* data) override;
  void onDisconnect(seasocks::WebSocket* connection) override;

  void SendData(const std::string &data);

 private:
  std::set<seasocks::WebSocket *> connections_;
};

WebsocketHandler::WebsocketHandler() {
}

void WebsocketHandler::onConnect(seasocks::WebSocket *connection) {
  connections_.insert(connection);
  LOG(INFO, "Connected: %s : %s\n", connection->getRequestUri().c_str(),
      seasocks::formatAddress(connection->getRemoteAddress()).c_str());
}

void WebsocketHandler::onData(seasocks::WebSocket * /*connection*/,
                              const char *data) {
  LOG(INFO, "Got data: %s\n", data);
}

void WebsocketHandler::onDisconnect(seasocks::WebSocket *connection) {
  connections_.erase(connection);
  LOG(INFO, "Disconnected: %s : %s\n", connection->getRequestUri().c_str(),
      seasocks::formatAddress(connection->getRemoteAddress()).c_str());
}

void WebsocketHandler::SendData(const std::string &data) {
  for (seasocks::WebSocket *websocket : connections_) {
    websocket->send(reinterpret_cast<const uint8_t *>(data.data()),
                    data.size());
  }
}

// TODO(Brian): Put this somewhere shared.
class SeasocksLogger : public seasocks::PrintfLogger {
 public:
  SeasocksLogger(Level min_level_to_log) : PrintfLogger(min_level_to_log) {}
  void log(Level level, const char* message) override;
};

void SeasocksLogger::log(Level level, const char *message) {
  // Convert Seasocks error codes to AOS.
  log_level aos_level;
  switch (level) {
    case seasocks::Logger::INFO:
      aos_level = INFO;
      break;
    case seasocks::Logger::WARNING:
      aos_level = WARNING;
      break;
    case seasocks::Logger::ERROR:
    case seasocks::Logger::SEVERE:
      aos_level = ERROR;
      break;
    case seasocks::Logger::DEBUG:
    case seasocks::Logger::ACCESS:
    default:
      aos_level = DEBUG;
      break;
  }
  LOG(aos_level, "Seasocks: %s\n", message);
}

struct LocalCameraTarget {
  double x, y, theta;
};

struct LocalCameraFrame {
  aos::monotonic_clock::time_point capture_time =
      aos::monotonic_clock::min_time;
  std::vector<LocalCameraTarget> targets;

  bool IsValid(aos::monotonic_clock::time_point now) {
    return capture_time + std::chrono::seconds(1) > now;
  }
};

// Sends a new chunk of data to all the websocket clients.
class UpdateData : public seasocks::Server::Runnable {
 public:
  UpdateData(WebsocketHandler *websocket_handler, std::string &&data)
      : websocket_handler_(websocket_handler), data_(std::move(data)) {}
  ~UpdateData() override = default;
  UpdateData(const UpdateData &) = delete;
  UpdateData &operator=(const UpdateData &) = delete;

  void run() override { websocket_handler_->SendData(data_); }

 private:
  WebsocketHandler *const websocket_handler_;
  const std::string data_;
};

void DataThread(seasocks::Server *server, WebsocketHandler *websocket_handler) {
  auto &camera_frames = y2019::control_loops::drivetrain::camera_frames;
  auto &drivetrain_status = frc971::control_loops::drivetrain_queue.status;

  std::array<LocalCameraFrame, 5> latest_frames;
  aos::monotonic_clock::time_point last_send_time = aos::monotonic_clock::now();

  while (true) {
    camera_frames.FetchNextBlocking();
    drivetrain_status.FetchLatest();
    if (!drivetrain_status.get()) {
      // Try again if we don't have any drivetrain statuses.
      continue;
    }

    {
      const auto &new_frame = *camera_frames;
      if (new_frame.camera < latest_frames.size()) {
        latest_frames[new_frame.camera].capture_time =
            aos::monotonic_clock::time_point(
                std::chrono::nanoseconds(new_frame.timestamp));
        latest_frames[new_frame.camera].targets.clear();
        for (int target = 0; target < new_frame.num_targets; ++target) {
          latest_frames[new_frame.camera].targets.emplace_back();
          // TODO: Do something useful.
        }
      }
    }

    const auto now = aos::monotonic_clock::now();
    if (now > last_send_time + std::chrono::milliseconds(100)) {
      last_send_time = now;
      std::ostringstream stream;
      stream << "{\n";

      stream << "\"robot\": {";
      stream << "\"x\": " << drivetrain_status->x << ",";
      stream << "\"y\": " << drivetrain_status->y << ",";
      stream << "\"theta\": " << drivetrain_status->theta;
      stream << "}\n";

      stream << "}";
      server->execute(
          std::make_shared<UpdateData>(websocket_handler, stream.str()));
    }
  }
}

}  // namespace vision
}  // namespace y2019

int main(int, char *[]) {
  // Make sure to reference this to force the linker to include it.
  findEmbeddedContent("");

  aos::InitNRT();

  seasocks::Server server(::std::shared_ptr<seasocks::Logger>(
      new y2019::vision::SeasocksLogger(seasocks::Logger::INFO)));

  auto websocket_handler = std::make_shared<y2019::vision::WebsocketHandler>();
  server.addWebSocketHandler("/ws", websocket_handler);

  std::thread data_thread{[&server, websocket_handler]() {
    y2019::vision::DataThread(&server, websocket_handler.get());
  }};

  // See if we are on a robot.  If so, serve the robot www folder.
  bool serve_www = false;
  {
    struct stat result;
    if (stat("/home/admin/robot_code/www", &result) == 0) {
      serve_www = true;
    }
  }

  server.serve(
      serve_www ? "/home/admin/robot_code/www" : "y2019/vision/server/www",
      1180);

  return 0;
}
