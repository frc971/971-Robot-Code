#include <map>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <random>

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/time/time.h"
#include "frc971/vision/vision_generated.h"
#include "y2022/vision/blob_detector.h"
#include "y2022/vision/target_estimate_generated.h"

DEFINE_string(capture, "",
              "If set, capture a single image and save it to this filename.");
DEFINE_string(channel, "/camera", "Channel name for the image.");
DEFINE_string(config, "aos_config.json", "Path to the config file to use.");
DEFINE_string(png_dir, "", "Path to a set of images to display.");
DEFINE_bool(show_features, true, "Show the blobs.");

namespace y2022 {
namespace vision {
namespace {

aos::Fetcher<frc971::vision::CameraImage> image_fetcher;
aos::Fetcher<y2022::vision::TargetEstimate> target_estimate_fetcher;

std::vector<std::vector<cv::Point>> FbsToCvBlobs(
    const flatbuffers::Vector<flatbuffers::Offset<Blob>> &blobs_fbs) {
  std::vector<std::vector<cv::Point>> blobs;
  for (const auto blob : blobs_fbs) {
    std::vector<cv::Point> points;
    for (const Point *point : *blob->points()) {
      points.emplace_back(cv::Point{point->x(), point->y()});
    }
    blobs.emplace_back(points);
  }
  return blobs;
}

std::vector<BlobDetector::BlobStats> FbsToBlobStats(
    const flatbuffers::Vector<flatbuffers::Offset<BlobStatsFbs>>
        &blob_stats_fbs) {
  std::vector<BlobDetector::BlobStats> blob_stats;
  for (const auto stats_fbs : blob_stats_fbs) {
    cv::Point centroid{stats_fbs->centroid()->x(), stats_fbs->centroid()->y()};
    blob_stats.emplace_back(BlobDetector::BlobStats{
        centroid, stats_fbs->aspect_ratio(), stats_fbs->area(),
        static_cast<size_t>(stats_fbs->num_points())});
  }
  return blob_stats;
}

bool DisplayLoop() {
  int64_t image_timestamp = 0;
  const frc971::vision::CameraImage *image;
  // Read next image
  if (!image_fetcher.Fetch()) {
    LOG(INFO) << "Couldn't fetch image";
    return true;
  }

  image = image_fetcher.get();
  CHECK(image != nullptr) << "Couldn't read image";
  image_timestamp = image->monotonic_timestamp_ns();
  VLOG(2) << "Got image at timestamp: " << image_timestamp;

  // TODO(Milind) Store the target estimates and match them by timestamp to make
  // sure we're getting the right one.
  const TargetEstimate *target_est = nullptr;
  if (target_estimate_fetcher.Fetch()) {
    target_est = target_estimate_fetcher.get();
  }

  // Create color image:
  cv::Mat image_color_mat(cv::Size(image->cols(), image->rows()), CV_8UC2,
                          (void *)image->data()->data());
  cv::Mat bgr_image(cv::Size(image->cols(), image->rows()), CV_8UC3);
  cv::cvtColor(image_color_mat, bgr_image, cv::COLOR_YUV2BGR_YUYV);

  if (!FLAGS_capture.empty()) {
    cv::imwrite(FLAGS_capture, bgr_image);
    return false;
  }

  LOG(INFO) << image->monotonic_timestamp_ns() << ": # unfiltered blobs: "
            << target_est->blob_result()->unfiltered_blobs()->size()
            << "; # filtered blobs: "
            << target_est->blob_result()->filtered_blobs()->size();

  cv::Mat ret_image(cv::Size(image->cols(), image->rows()), CV_8UC3);
  if (target_est != nullptr) {
    BlobDetector::DrawBlobs(
        ret_image, FbsToCvBlobs(*target_est->blob_result()->filtered_blobs()),
        FbsToCvBlobs(*target_est->blob_result()->unfiltered_blobs()),
        FbsToBlobStats(*target_est->blob_result()->blob_stats()),
        cv::Point{target_est->blob_result()->centroid()->x(),
                  target_est->blob_result()->centroid()->y()});
    cv::imshow("blobs", ret_image);
  }

  cv::imshow("image", bgr_image);

  int keystroke = cv::waitKey(1);
  if ((keystroke & 0xFF) == static_cast<int>('c')) {
    // Convert again, to get clean image
    cv::cvtColor(image_color_mat, bgr_image, cv::COLOR_YUV2BGR_YUYV);
    std::stringstream name;
    name << "capture-" << aos::realtime_clock::now() << ".png";
    cv::imwrite(name.str(), bgr_image);
    LOG(INFO) << "Saved image file: " << name.str();
  } else if ((keystroke & 0xFF) == static_cast<int>('q')) {
    return false;
  }
  return true;
}

void ViewerMain() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  image_fetcher =
      event_loop.MakeFetcher<frc971::vision::CameraImage>(FLAGS_channel);

  target_estimate_fetcher =
      event_loop.MakeFetcher<y2022::vision::TargetEstimate>(FLAGS_channel);

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

  image_fetcher = aos::Fetcher<frc971::vision::CameraImage>();
}
}  // namespace
}  // namespace vision
}  // namespace y2022

// Quick and lightweight viewer for images
int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2022::vision::ViewerMain();
}
