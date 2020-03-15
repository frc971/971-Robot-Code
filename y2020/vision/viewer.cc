#include <map>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/time/time.h"
#include "y2020/vision/sift/sift_generated.h"
#include "y2020/vision/vision_generated.h"

DEFINE_string(config, "config.json", "Path to the config file to use.");
DEFINE_string(channel, "/camera", "Channel name for the image.");

namespace frc971 {
namespace vision {
namespace {

void ViewerMain() {
  struct TargetData {
    float x;
    float y;
    float radius;
  };

  std::map<int64_t, TargetData> target_data_map;

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  event_loop.MakeWatcher(
      FLAGS_channel, [&target_data_map](const CameraImage &image) {
        // Create color image:
        cv::Mat image_color_mat(cv::Size(image.cols(), image.rows()), CV_8UC2,
                                (void *)image.data()->data());
        cv::Mat rgb_image(cv::Size(image.cols(), image.rows()), CV_8UC3);
        cv::cvtColor(image_color_mat, rgb_image, CV_YUV2BGR_YUYV);

        unsigned long timestamp = image.monotonic_timestamp_ns();
        auto target_it = target_data_map.find(timestamp);
        if (target_it != target_data_map.end()) {
          float x = target_it->second.x;
          float y = target_it->second.y;
          float radius = target_it->second.radius;
          cv::circle(rgb_image, cv::Point2f(x, y), radius,
                     cv::Scalar(0, 255, 0), 5);
        }

        cv::imshow("Display", rgb_image);
        int keystroke = cv::waitKey(1);
        if ((keystroke & 0xFF) == static_cast<int>('c')) {
          // Convert again, to get clean image
          cv::cvtColor(image_color_mat, rgb_image, CV_YUV2BGR_YUYV);
          std::stringstream name;
          name << "capture-" << aos::realtime_clock::now() << ".png";
          cv::imwrite(name.str(), rgb_image);
          LOG(INFO) << "Saved image file: " << name.str();
        } else if ((keystroke & 0xFF) == static_cast<int>('q')) {
          exit(0);
        }
      });

  event_loop.MakeWatcher(
      FLAGS_channel, [&target_data_map](const sift::ImageMatchResult &match) {
        int64_t timestamp = match.image_monotonic_timestamp_ns();
        if (match.camera_poses() != NULL && match.camera_poses()->size() > 0) {
          LOG(INFO) << "Got match!\n";
          TargetData target_data = {
              match.camera_poses()->Get(0)->query_target_point_x(),
              match.camera_poses()->Get(0)->query_target_point_y(),
              match.camera_poses()->Get(0)->query_target_point_radius()};
          target_data_map[timestamp] = target_data;
          while (target_data_map.size() > 10u) {
            target_data_map.erase(target_data_map.begin());
          }
        }
      });

  event_loop.Run();
}

}  // namespace
}  // namespace vision
}  // namespace frc971

// Quick and lightweight grayscale viewer for images
int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc971::vision::ViewerMain();
}
