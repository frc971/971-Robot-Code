#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <array>
#include <cmath>
#include <memory>
#include <set>
#include <sstream>

#include "aos/containers/ring_buffer.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/seasocks/seasocks_logger.h"
#include "aos/time/time.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/control_loops/pose.h"
#include "google/protobuf/util/json_util.h"
#include "internal/Embedded.h"
#include "seasocks/Server.h"
#include "seasocks/StringUtil.h"
#include "seasocks/WebSocket.h"
#include "y2019/constants.h"
#include "y2019/control_loops/drivetrain/camera_generated.h"
#include "y2019/control_loops/superstructure/superstructure_status_generated.h"
#include "y2019/vision/server/server_data.pb.h"

namespace y2019 {
namespace vision {

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

void WebsocketHandler::onData(seasocks::WebSocket * /*connection*/,
                              const char *data) {
  AOS_LOG(INFO, "Got data: %s\n", data);
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

struct LocalCameraTarget {
  double x, y, theta;
};

struct LocalCameraFrame {
  aos::monotonic_clock::time_point capture_time =
      aos::monotonic_clock::min_time;
  std::vector<LocalCameraTarget> targets;

  bool IsValid(aos::monotonic_clock::time_point now) {
    return capture_time + chrono::seconds(1) > now;
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

struct DrivetrainPosition {
  ::aos::monotonic_clock::time_point time;
  double x;
  double y;
  double theta;
};

DrivetrainPosition ComputePosition(
    const ::aos::RingBuffer<DrivetrainPosition, 200> &data,
    ::aos::monotonic_clock::time_point time) {
  DrivetrainPosition drivetrain_now{time, 0.0f, 0.0f, 0.0f};

  const size_t after_index = ::std::max(
      static_cast<size_t>(1),
      static_cast<size_t>(::std::distance(
          data.begin(),
          ::std::lower_bound(
              data.begin(), data.end(), drivetrain_now,
              [](const DrivetrainPosition &a, const DrivetrainPosition &b) {
                return a.time < b.time;
              }))));
  const size_t before_index = after_index - 1;

  const DrivetrainPosition &before = data[before_index];
  const DrivetrainPosition &after = data[after_index];

  double alpha = static_cast<double>((time - before.time).count()) /
                 static_cast<double>((after.time - before.time).count());
  drivetrain_now.x = (1.0 - alpha) * before.x + alpha * after.x;
  drivetrain_now.y = (1.0 - alpha) * before.y + alpha * after.y;
  drivetrain_now.theta = (1.0 - alpha) * before.theta + alpha * after.theta;

  return drivetrain_now;
}

void DataThread(seasocks::Server *server, WebsocketHandler *websocket_handler) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());

  ::aos::Fetcher<::frc971::control_loops::drivetrain::Status>
      drivetrain_status_fetcher =
          event_loop.MakeFetcher<::frc971::control_loops::drivetrain::Status>(
              "/drivetrain");

  ::aos::Fetcher<::y2019::control_loops::superstructure::Status>
      superstructure_status_fetcher =
          event_loop
              .MakeFetcher<::y2019::control_loops::superstructure::Status>(
                  "/superstructure");

  ::std::array<LocalCameraFrame, 5> latest_frames;
  ::std::array<aos::monotonic_clock::time_point, 5> last_target_time;
  last_target_time.fill(aos::monotonic_clock::epoch());
  aos::monotonic_clock::time_point last_send_time = aos::monotonic_clock::now();
  DebugData debug_data;
  ::aos::RingBuffer<DrivetrainPosition, 200> drivetrain_log;

  event_loop.MakeWatcher(
      "/camera", [websocket_handler, server, &latest_frames, &last_target_time,
                  &drivetrain_status_fetcher, &superstructure_status_fetcher,
                  &last_send_time, &drivetrain_log, &debug_data](
                     const ::y2019::control_loops::drivetrain::CameraFrame
                         &camera_frames) {
        while (drivetrain_status_fetcher.FetchNext()) {
          DrivetrainPosition drivetrain_position{
              drivetrain_status_fetcher.context().monotonic_event_time,
              drivetrain_status_fetcher->x(), drivetrain_status_fetcher->y(),
              drivetrain_status_fetcher->theta()};

          drivetrain_log.Push(drivetrain_position);
        }
        superstructure_status_fetcher.Fetch();
        if (!drivetrain_status_fetcher.get() ||
            !superstructure_status_fetcher.get()) {
          // Try again if we don't have any drivetrain statuses.
          return;
        }
        const auto now = aos::monotonic_clock::now();

        {
          const auto &new_frame = camera_frames;
          // TODO(james): Maybe we only want to fill out a new frame if it has
          // targets or the saved target is > 0.1 sec old? Not sure, but for now
          if (new_frame.camera() < latest_frames.size()) {
            latest_frames[new_frame.camera()].capture_time =
                aos::monotonic_clock::time_point(
                    chrono::nanoseconds(new_frame.timestamp()));
            latest_frames[new_frame.camera()].targets.clear();
            if (new_frame.has_targets() && new_frame.targets()->size() > 0) {
              last_target_time[new_frame.camera()] =
                  latest_frames[new_frame.camera()].capture_time;
            }
            for (const control_loops::drivetrain::CameraTarget *target :
                 *new_frame.targets()) {
              latest_frames[new_frame.camera()].targets.emplace_back();
              const float heading = target->heading();
              const float distance = target->distance();
              latest_frames[new_frame.camera()].targets.back().x =
                  ::std::cos(heading) * distance;
              latest_frames[new_frame.camera()].targets.back().y =
                  ::std::sin(heading) * distance;
              latest_frames[new_frame.camera()].targets.back().theta =
                  target->skew();
            }
          }
        }

        for (size_t ii = 0; ii < latest_frames.size(); ++ii) {
          CameraDebug *camera_debug = debug_data.add_camera_debug();
          LocalCameraFrame cur_frame = latest_frames[ii];
          constants::Values::CameraCalibration camera_info =
              constants::GetValues().cameras[ii];
          frc971::control_loops::TypedPose<double> camera_pose =
              camera_info.pose;

          const DrivetrainPosition robot_position =
              ComputePosition(drivetrain_log, cur_frame.capture_time);
          const ::frc971::control_loops::TypedPose<double> robot_pose(
              {robot_position.x, robot_position.y, 0}, robot_position.theta);

          camera_pose.set_base(&robot_pose);

          camera_debug->set_current_frame_age(
              ::aos::time::DurationInSeconds(now - cur_frame.capture_time));
          camera_debug->set_time_since_last_target(
              ::aos::time::DurationInSeconds(now - last_target_time[ii]));
          for (const auto &target : cur_frame.targets) {
            frc971::control_loops::TypedPose<double> target_pose(
                &camera_pose, {target.x, target.y, 0}, target.theta);
            Pose *pose = camera_debug->add_targets();
            pose->set_x(target_pose.abs_pos().x());
            pose->set_y(target_pose.abs_pos().y());
            pose->set_theta(target_pose.abs_theta());
          }
        }

        if (now > last_send_time + chrono::milliseconds(100)) {
          last_send_time = now;
          debug_data.mutable_robot_pose()->set_x(drivetrain_status_fetcher->x());
          debug_data.mutable_robot_pose()->set_y(drivetrain_status_fetcher->y());
          debug_data.mutable_robot_pose()->set_theta(
              drivetrain_status_fetcher->theta());
          {
            LineFollowDebug *line_debug =
                debug_data.mutable_line_follow_debug();
            line_debug->set_frozen(
                drivetrain_status_fetcher->line_follow_logging()->frozen());
            line_debug->set_have_target(
                drivetrain_status_fetcher->line_follow_logging()->have_target());
            line_debug->mutable_goal_target()->set_x(
                drivetrain_status_fetcher->line_follow_logging()->x());
            line_debug->mutable_goal_target()->set_y(
                drivetrain_status_fetcher->line_follow_logging()->y());
            line_debug->mutable_goal_target()->set_theta(
                drivetrain_status_fetcher->line_follow_logging()->theta());
          }

          Sensors *sensors = debug_data.mutable_sensors();
          sensors->set_wrist(
              superstructure_status_fetcher->wrist()->position());
          sensors->set_elevator(
              superstructure_status_fetcher->elevator()->position());
          sensors->set_intake(superstructure_status_fetcher->intake()->position());
          sensors->set_stilts(superstructure_status_fetcher->stilts()->position());
          sensors->set_has_piece(superstructure_status_fetcher->has_piece());

          ::std::string json;
          google::protobuf::util::MessageToJsonString(debug_data, &json);
          server->execute(std::make_shared<UpdateData>(websocket_handler,
                                                       ::std::move(json)));

          debug_data.Clear();
        }
      });
  event_loop.Run();
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
