#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/input/joystick_state_generated.h"
#include "frc971/vision/vision_generated.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "y2022/control_loops/superstructure/superstructure_status_generated.h"
#include "y2022/vision/blob_detector.h"

DEFINE_string(pi, "pi3", "Node name to replay.");
DEFINE_string(image_save_prefix, "/tmp/img",
              "Prefix to use for saving images from the logfile.");
DEFINE_bool(display, false, "If true, display the images with a timeout.");
DEFINE_bool(detected_only, false,
            "If true, only write images which had blobs (unfiltered) detected");
DEFINE_bool(filtered_only, false,
            "If true, only write images which had blobs (filtered) detected");
DEFINE_bool(match_timestamps, false,
            "If true, name the files based on the time since the robot was "
            "enabled (match start). Only consider images during this time");
DEFINE_string(logger_pi_log, "/tmp/logger_pi/", "Path to logger pi log");
DEFINE_string(roborio_log, "/tmp/roborio/", "Path to roborio log");

namespace y2022 {
namespace vision {
namespace {

using aos::monotonic_clock;
namespace superstructure = control_loops::superstructure;

// Information to extract from the roborio log
struct ReplayData {
  monotonic_clock::time_point match_start;
  monotonic_clock::time_point match_end;
  std::map<monotonic_clock::time_point, bool> robot_moving;
  std::map<monotonic_clock::time_point, superstructure::SuperstructureState>
      superstructure_states;
};

// Extract the useful data from the roborio log to be used for naming images
void ReplayRoborio(ReplayData *data) {
  data->match_start = monotonic_clock::min_time;
  data->match_end = monotonic_clock::min_time;

  std::vector<std::string> unsorted_logfiles =
      aos::logger::FindLogs(FLAGS_roborio_log);
  // Open logfiles
  aos::logger::LogReader reader(aos::logger::SortParts(unsorted_logfiles));
  reader.Register();
  const aos::Node *roborio =
      aos::configuration::GetNode(reader.configuration(), "roborio");

  std::unique_ptr<aos::EventLoop> event_loop =
      reader.event_loop_factory()->MakeEventLoop("roborio", roborio);

  auto joystick_state_fetcher =
      event_loop->MakeFetcher<aos::JoystickState>("/roborio/aos");
  auto drivetrain_status_fetcher =
      event_loop->MakeFetcher<frc971::control_loops::drivetrain::Status>(
          "/drivetrain");
  auto superstructure_status_fetcher =
      event_loop->MakeFetcher<superstructure::Status>("/superstructure");

  // Periodically check if the robot state updated
  event_loop->AddPhasedLoop(
      [&](int) {
        // Find the match start and end times
        if (joystick_state_fetcher.Fetch()) {
          if (data->match_start == monotonic_clock::min_time &&
              joystick_state_fetcher->enabled()) {
            data->match_start =
                joystick_state_fetcher.context().monotonic_event_time;
          } else {
            if (data->match_end == monotonic_clock::min_time &&
                data->match_start != monotonic_clock::min_time &&
                !joystick_state_fetcher->autonomous() &&
                !joystick_state_fetcher->enabled()) {
              data->match_end =
                  joystick_state_fetcher.context().monotonic_event_time;
            }
          }
        }

        // Add whether the robot was moving at a non-negligible speed to
        // the image name for debugging.
        drivetrain_status_fetcher.Fetch();
        if (drivetrain_status_fetcher.get()) {
          // If the robot speed was atleast this (m/s), it is
          // considered moving.
          constexpr double kMinMovingRobotSpeed = 0.5;
          data->robot_moving[drivetrain_status_fetcher.context()
                                 .monotonic_event_time] =
              (std::abs(drivetrain_status_fetcher->robot_speed()) >=
               kMinMovingRobotSpeed);
        }

        superstructure_status_fetcher.Fetch();
        if (superstructure_status_fetcher.get()) {
          data->superstructure_states[superstructure_status_fetcher.context()
                                          .monotonic_event_time] =
              superstructure_status_fetcher->state();
        }
      },
      std::chrono::milliseconds(50));
  reader.event_loop_factory()->Run();
}

template <typename T>
T ClosestElement(const std::map<monotonic_clock::time_point, T> &map,
                 monotonic_clock::time_point now) {
  T closest;

  // The closest element is either the one right above it, or the element before
  // that one
  auto closest_it = map.lower_bound(now);
  // Handle the case where now is greater than all times in the map
  if (closest_it == map.cend()) {
    closest_it--;
    closest = closest_it->second;
  } else {
    // Start off with the closest as the first after now
    closest = closest_it->second;
    const monotonic_clock::duration after_duration = closest_it->first - now;
    closest_it--;

    // If there is a time before, check if that's closer to now
    if (closest_it != map.cbegin()) {
      const monotonic_clock::duration before_duration = now - closest_it->first;
      if (before_duration < after_duration) {
        closest = closest_it->second;
      }
    }
  }

  return closest;
}

// Extract images from the pi logs
void ReplayPi(const ReplayData &data) {
  if (FLAGS_match_timestamps) {
    CHECK_NE(data.match_start, monotonic_clock::min_time)
        << "Can't use match timestamps if match never started";
    CHECK_NE(data.match_end, monotonic_clock::min_time)
        << "Can't use match timestamps if match never ended";
  }

  std::vector<std::string> unsorted_logfiles =
      aos::logger::FindLogs(FLAGS_logger_pi_log);

  // Open logfiles
  aos::logger::LogReader reader(aos::logger::SortParts(unsorted_logfiles));
  reader.Register();
  const aos::Node *pi =
      aos::configuration::GetNode(reader.configuration(), FLAGS_pi);

  std::unique_ptr<aos::EventLoop> event_loop =
      reader.event_loop_factory()->MakeEventLoop("player", pi);

  LOG(INFO) << "Match start: " << data.match_start
            << ", match end: " << data.match_end;

  size_t nonmatch_image_count = 0;

  event_loop->MakeWatcher(
      "/camera/decimated", [&](const frc971::vision::CameraImage &image) {
        const auto match_start = data.match_start;
        // Find the closest robot moving and superstructure state to now
        const bool robot_moving =
            ClosestElement(data.robot_moving, event_loop->monotonic_now());
        const auto superstructure_state = ClosestElement(
            data.superstructure_states, event_loop->monotonic_now());

        if (FLAGS_match_timestamps) {
          if (event_loop->monotonic_now() < data.match_start) {
            // Ignore prematch images if we only care about ones during the
            // match
            return;
          } else if (event_loop->monotonic_now() >= data.match_end) {
            // We're done if the match is over and we only wanted match images
            reader.event_loop_factory()->Exit();
            return;
          }
        }

        // Create color image:
        cv::Mat image_color_mat(cv::Size(image.cols(), image.rows()), CV_8UC2,
                                (void *)image.data()->data());
        cv::Mat image_mat(cv::Size(image.cols(), image.rows()), CV_8UC3);
        cv::cvtColor(image_color_mat, image_mat, cv::COLOR_YUV2BGR_YUYV);

        bool use_image = true;
        if (FLAGS_detected_only || FLAGS_filtered_only) {
          // TODO(milind): if adding target estimation here in the future,
          // undistortion is needed
          BlobDetector::BlobResult blob_result;
          BlobDetector::ExtractBlobs(image_mat, &blob_result);

          use_image =
              ((FLAGS_filtered_only ? blob_result.filtered_blobs.size()
                                    : blob_result.unfiltered_blobs.size()) > 0);
        }

        if (use_image) {
          if (!FLAGS_image_save_prefix.empty()) {
            std::stringstream image_name;
            image_name << FLAGS_image_save_prefix;

            if (FLAGS_match_timestamps) {
              // Add the time since match start into the image for debugging.
              // We can match images with the game recording.
              image_name << "match_"
                         << std::chrono::duration_cast<std::chrono::seconds>(
                                event_loop->monotonic_now() - match_start)
                                .count()
                         << 's';
            } else {
              image_name << nonmatch_image_count++;
            }

            // Add superstructure state to the filename
            if (superstructure_state !=
                superstructure::SuperstructureState::IDLE) {
              std::string superstructure_state_name =
                  superstructure::EnumNameSuperstructureState(
                      superstructure_state);
              std::transform(superstructure_state_name.begin(),
                             superstructure_state_name.end(),
                             superstructure_state_name.begin(),
                             [](char c) { return std::tolower(c); });
              image_name << '_' << superstructure_state_name;
            }

            if (robot_moving) {
              image_name << "_moving";
            }

            image_name << ".png";

            cv::imwrite(image_name.str(), image_mat);
          }
          if (FLAGS_display) {
            cv::imshow("Display", image_mat);
            cv::waitKey(FLAGS_detected_only || FLAGS_filtered_only ? 10 : 1);
          }
        }
      });

  reader.event_loop_factory()->Run();
}

void ViewerMain() {
  ReplayData data;
  ReplayRoborio(&data);
  ReplayPi(data);
}

}  // namespace
}  // namespace vision
}  // namespace y2022

// Quick and lightweight viewer for image logs
int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2022::vision::ViewerMain();
}
