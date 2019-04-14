#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <array>
#include <cmath>
#include <memory>
#include <set>
#include <sstream>

#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/seasocks/seasocks_logger.h"
#include "aos/time/time.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/pose.h"
#include "google/protobuf/util/json_util.h"
#include "internal/Embedded.h"
#include "seasocks/Server.h"
#include "seasocks/StringUtil.h"
#include "seasocks/WebSocket.h"
#include "y2019/constants.h"
#include "y2019/control_loops/drivetrain/camera.q.h"
#include "y2019/vision/server/server_data.pb.h"

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
  std::array<aos::monotonic_clock::time_point, 5> last_target_time;
  last_target_time.fill(aos::monotonic_clock::epoch());
  aos::monotonic_clock::time_point last_send_time = aos::monotonic_clock::now();

  while (true) {
    camera_frames.FetchNextBlocking();
    drivetrain_status.FetchLatest();
    if (!drivetrain_status.get()) {
      // Try again if we don't have any drivetrain statuses.
      continue;
    }
    const auto now = aos::monotonic_clock::now();

    {
      const auto &new_frame = *camera_frames;
      // TODO(james): Maybe we only want to fill out a new frame if it has
      // targets or the saved target is > 0.1 sec old? Not sure, but for now
      if (new_frame.camera < latest_frames.size()) {
        latest_frames[new_frame.camera].capture_time =
            aos::monotonic_clock::time_point(
                std::chrono::nanoseconds(new_frame.timestamp));
        latest_frames[new_frame.camera].targets.clear();
        if (new_frame.num_targets > 0) {
          last_target_time[new_frame.camera] =
              latest_frames[new_frame.camera].capture_time;
        }
        for (int target = 0; target < new_frame.num_targets; ++target) {
          latest_frames[new_frame.camera].targets.emplace_back();
          const float heading = new_frame.targets[target].heading;
          const float distance = new_frame.targets[target].distance;
          latest_frames[new_frame.camera].targets.back().x =
              ::std::cos(heading) * distance;
          latest_frames[new_frame.camera].targets.back().y =
              ::std::sin(heading) * distance;
          latest_frames[new_frame.camera].targets.back().theta =
              new_frame.targets[target].skew;
        }
      }
    }

    if (now > last_send_time + std::chrono::milliseconds(100)) {
      // TODO(james): Use protobuf or the such to generate JSON rather than
      // doing so manually.
      last_send_time = now;
      DebugData debug_data;
      debug_data.mutable_robot_pose()->set_x(drivetrain_status->x);
      debug_data.mutable_robot_pose()->set_y(drivetrain_status->y);
      debug_data.mutable_robot_pose()->set_theta(drivetrain_status->theta);
      frc971::control_loops::TypedPose<double> robot_pose(
          {drivetrain_status->x, drivetrain_status->y, 0},
          drivetrain_status->theta);
      {
        LineFollowDebug *line_debug = debug_data.mutable_line_follow_debug();
        line_debug->set_frozen(drivetrain_status->line_follow_logging.frozen);
        line_debug->set_have_target(
            drivetrain_status->line_follow_logging.have_target);
        line_debug->mutable_goal_target()->set_x(
            drivetrain_status->line_follow_logging.x);
        line_debug->mutable_goal_target()->set_y(
            drivetrain_status->line_follow_logging.y);
        line_debug->mutable_goal_target()->set_theta(
            drivetrain_status->line_follow_logging.theta);
      }
      for (size_t ii = 0; ii < latest_frames.size(); ++ii) {
        CameraDebug *camera_debug = debug_data.add_camera_debug();
        LocalCameraFrame cur_frame = latest_frames[ii];
        constants::Values::CameraCalibration camera_info =
            constants::GetValues().cameras[ii];
        frc971::control_loops::TypedPose<double> camera_pose = camera_info.pose;
        camera_pose.set_base(&robot_pose);

        camera_debug->set_current_frame_age(
            ::std::chrono::duration_cast<::std::chrono::duration<double>>(
                now - cur_frame.capture_time).count());
        camera_debug->set_time_since_last_target(
            ::std::chrono::duration_cast<::std::chrono::duration<double>>(
                now - last_target_time[ii]).count());
        for (const auto &target : cur_frame.targets) {
          frc971::control_loops::TypedPose<double> target_pose(
              &camera_pose, {target.x, target.y, 0}, target.theta);
          Pose *pose = camera_debug->add_targets();
          pose->set_x(target_pose.abs_pos().x());
          pose->set_y(target_pose.abs_pos().y());
          pose->set_theta(target_pose.abs_theta());
        }
      }
      ::std::string json;
      google::protobuf::util::MessageToJsonString(debug_data, &json);
      server->execute(
          std::make_shared<UpdateData>(websocket_handler, ::std::move(json)));
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
      new ::aos::seasocks::SeasocksLogger(seasocks::Logger::Level::Info)));

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
