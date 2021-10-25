#include <map>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <random>

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/time/time.h"
#include "y2020/vision/sift/sift_generated.h"
#include "y2020/vision/vision_generated.h"

DEFINE_string(config, "config.json", "Path to the config file to use.");
DEFINE_bool(show_features, true, "Show the SIFT features that matched.");
DEFINE_string(channel, "/camera", "Channel name for the image.");

namespace frc971 {
namespace vision {
namespace {

aos::Fetcher<CameraImage> image_fetcher;
aos::Fetcher<sift::ImageMatchResult> match_fetcher;
cv::Mat palette_;

bool DisplayLoop() {
  // Try to get target match data
  int64_t match_timestamp = 0;
  const sift::ImageMatchResult *match;
  if (match_fetcher.Fetch()) {
    match = match_fetcher.get();
    CHECK(match != nullptr) << "Got null when trying to fetch match result";

    match_timestamp = match->image_monotonic_timestamp_ns();
    if (match->camera_poses() != nullptr && match->camera_poses()->size() > 0) {
      VLOG(2) << "Got matches for timestamp " << match_timestamp << "\n";
    }
  } else {
    VLOG(2) << "Didn't get match this cycle";
  }

  int64_t image_timestamp = 0;
  bool matching_image_found = false;
  const CameraImage *image;
  while (!matching_image_found) {
    // Read next image
    if (!image_fetcher.FetchNext()) {
      VLOG(2) << "Couldn't fetch next image";
      return true;
    }

    image = image_fetcher.get();
    CHECK(image != nullptr) << "Couldn't read image";
    image_timestamp = image->monotonic_timestamp_ns();
    VLOG(2) << "Got image at timestamp: " << image_timestamp;

    if (match_timestamp != 0 && image_timestamp == match_timestamp) {
      matching_image_found = true;
    } else if (image_timestamp > match_timestamp) {
      LOG(INFO) << "Image timestamp went past match_timestamp";
      return true;
    }
  }

  // Create color image:
  cv::Mat image_color_mat(cv::Size(image->cols(), image->rows()), CV_8UC2,
                          (void *)image->data()->data());
  cv::Mat rgb_image(cv::Size(image->cols(), image->rows()), CV_8UC3);
  cv::cvtColor(image_color_mat, rgb_image, CV_YUV2BGR_YUYV);

  if (matching_image_found) {
    // Draw whatever matches we have
    if (match->camera_poses() != nullptr && match->camera_poses()->size() > 0) {
      // Draw target point in image
      float x = match->camera_poses()->Get(0)->query_target_point_x();
      float y = match->camera_poses()->Get(0)->query_target_point_y();
      float radius = match->camera_poses()->Get(0)->query_target_point_radius();
      cv::circle(rgb_image, cv::Point2f(x, y), radius, cv::Scalar(0, 255, 0),
                 5);
    }

    if (FLAGS_show_features && match->image_matches() != nullptr &&
        match->image_matches()->size() > 0) {
      // Iterate through matches and draw matched keypoints
      for (uint model_match_ind = 0;
           model_match_ind < match->image_matches()->size();
           model_match_ind++) {
        auto match_list =
            match->image_matches()->Get(model_match_ind)->matches();
        if (match_list != nullptr && match_list->size() > 0) {
          int train_image_ind =
              match->image_matches()->Get(model_match_ind)->train_image();
          VLOG(2) << "Got " << match_list->size() << " matches to model "
                  << train_image_ind;

          // Picking color from palette and drawing
          auto color = palette_.at<cv::Vec3b>(train_image_ind % palette_.cols);
          LOG(INFO) << "Using color " << color;
          for (uint i = 0; i < match_list->size(); i++) {
            uint query_feature_ind = match_list->Get(i)->query_feature();
            float kp_x = match->features()->Get(query_feature_ind)->x();
            float kp_y = match->features()->Get(query_feature_ind)->y();
            cv::circle(rgb_image, cv::Point2f(kp_x, kp_y), 5, color, 2);
          }
        }
      }
    }
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
    return false;
  }
  return true;
}

void ViewerMain() {
  // Create random color palette for distinguishing multiple models
  uchar colors[5][3] = {
      {0, 0, 255}, {0, 165, 255}, {0, 255, 255}, {255, 0, 0}, {128, 0, 128}};
  palette_ = cv::Mat(3, 5, CV_8U, &colors);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  image_fetcher = event_loop.MakeFetcher<CameraImage>(FLAGS_channel);

  // If we want to show the features, we have to use the detailed channel
  std::string result_channel = FLAGS_channel;
  if (FLAGS_show_features) {
    result_channel += "/detailed";
  }
  match_fetcher =
      event_loop.MakeFetcher<sift::ImageMatchResult>(result_channel);

  // Run the display loop
  event_loop.AddPhasedLoop(
      [&event_loop](int) {
        if (!DisplayLoop()) {
          LOG(INFO) << "Calling event_loop Exit";
          event_loop.Exit();
        };
      },
      ::std::chrono::milliseconds(100));

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
