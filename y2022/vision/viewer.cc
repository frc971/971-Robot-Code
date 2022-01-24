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

DEFINE_string(capture, "",
              "If set, capture a single image and save it to this filename.");
DEFINE_string(channel, "/camera", "Channel name for the image.");
DEFINE_string(config, "config.json", "Path to the config file to use.");
DEFINE_string(png_dir, "LED_Ring_exp", "Path to a set of images to display.");
DEFINE_bool(show_features, true, "Show the blobs.");

namespace y2022 {
namespace vision {
namespace {

aos::Fetcher<frc971::vision::CameraImage> image_fetcher;

bool DisplayLoop() {
  int64_t image_timestamp = 0;
  const frc971::vision::CameraImage *image;
  // Read next image
  if (!image_fetcher.FetchNext()) {
    VLOG(2) << "Couldn't fetch next image";
    return true;
  }

  image = image_fetcher.get();
  CHECK(image != nullptr) << "Couldn't read image";
  image_timestamp = image->monotonic_timestamp_ns();
  VLOG(2) << "Got image at timestamp: " << image_timestamp;

  // Create color image:
  cv::Mat image_color_mat(cv::Size(image->cols(), image->rows()), CV_8UC2,
                          (void *)image->data()->data());
  cv::Mat rgb_image(cv::Size(image->cols(), image->rows()), CV_8UC3);
  cv::cvtColor(image_color_mat, rgb_image, cv::COLOR_YUV2BGR_YUYV);

  if (!FLAGS_capture.empty()) {
    cv::imwrite(FLAGS_capture, rgb_image);
    return false;
  }

  cv::Mat binarized_image, ret_image;
  std::vector<std::vector<cv::Point>> unfiltered_blobs, filtered_blobs;
  std::vector<BlobDetector::BlobStats> blob_stats;
  BlobDetector::ExtractBlobs(rgb_image, binarized_image, ret_image,
                             filtered_blobs, unfiltered_blobs, blob_stats);

  LOG(INFO) << image->monotonic_timestamp_ns()
            << ": # blobs: " << filtered_blobs.size();

  // Downsize for viewing
  cv::resize(rgb_image, rgb_image,
             cv::Size(rgb_image.cols / 2, rgb_image.rows / 2),
             cv::INTER_LINEAR);

  cv::imshow("image", rgb_image);
  cv::imshow("blobs", ret_image);

  int keystroke = cv::waitKey(1);
  if ((keystroke & 0xFF) == static_cast<int>('c')) {
    // Convert again, to get clean image
    cv::cvtColor(image_color_mat, rgb_image, cv::COLOR_YUV2BGR_YUYV);
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
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  image_fetcher =
      event_loop.MakeFetcher<frc971::vision::CameraImage>(FLAGS_channel);

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

void ViewerLocal() {
  std::vector<cv::String> file_list;
  cv::glob(FLAGS_png_dir + "/*.png", file_list, false);
  for (auto file : file_list) {
    LOG(INFO) << "Reading file " << file;
    cv::Mat rgb_image = cv::imread(file.c_str());
    std::vector<std::vector<cv::Point>> filtered_blobs, unfiltered_blobs;
    std::vector<BlobDetector::BlobStats> blob_stats;
    cv::Mat binarized_image =
        cv::Mat::zeros(cv::Size(rgb_image.cols, rgb_image.rows), CV_8UC1);
    cv::Mat ret_image =
        cv::Mat::zeros(cv::Size(rgb_image.cols, rgb_image.rows), CV_8UC3);

    BlobDetector::ExtractBlobs(rgb_image, binarized_image, ret_image,
                               filtered_blobs, unfiltered_blobs, blob_stats);

    LOG(INFO) << ": # blobs: " << filtered_blobs.size() << " (# removed: "
              << unfiltered_blobs.size() - filtered_blobs.size() << ")";
    cv::imshow("image", rgb_image);
    cv::imshow("blobs", ret_image);

    int keystroke = cv::waitKey(0);
    if ((keystroke & 0xFF) == static_cast<int>('q')) {
      return;
    }
  }
}
}  // namespace
}  // namespace vision
}  // namespace y2022

// Quick and lightweight viewer for images
int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  if (FLAGS_png_dir != "")
    y2022::vision::ViewerLocal();
  else
    y2022::vision::ViewerMain();
}
