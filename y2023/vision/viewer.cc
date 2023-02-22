#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "absl/strings/match.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/time/time.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/vision/vision_generated.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "y2023/vision/april_debug_generated.h"
#include "y2023/vision/vision_util.h"

DEFINE_string(config, "aos_config.json", "Path to the config file to use.");
DEFINE_string(channel, "/camera", "Channel name for the image.");

DEFINE_string(capture, "",
              "If set, capture a single image and save it to this filename.");

DEFINE_int32(rate, 100, "Time in milliseconds to wait between images");

namespace y2023 {
namespace vision {
namespace {

using frc971::vision::CameraImage;

bool DisplayLoop(const cv::Mat intrinsics, const cv::Mat dist_coeffs,
                 aos::Fetcher<CameraImage> *image_fetcher,
                 aos::Fetcher<AprilDebug> *april_debug_fetcher) {
  const CameraImage *image;
  std::optional<const AprilDebug *> april_debug = std::nullopt;

  // Read next image
  if (!image_fetcher->Fetch()) {
    VLOG(2) << "Couldn't fetch next image";
    return true;
  }
  image = image_fetcher->get();
  CHECK(image != nullptr) << "Couldn't read image";

  if (april_debug_fetcher->Fetch()) {
    april_debug = april_debug_fetcher->get();
  } else {
    VLOG(2) << "Couldn't fetch next target map";
  }

  // Create color image:
  cv::Mat image_color_mat(cv::Size(image->cols(), image->rows()), CV_8UC2,
                          (void *)image->data()->data());
  cv::Mat bgr_image(cv::Size(image->cols(), image->rows()), CV_8UC3);
  cv::cvtColor(image_color_mat, bgr_image, cv::COLOR_YUV2BGR_YUYV);

  if (!FLAGS_capture.empty()) {
    if (absl::EndsWith(FLAGS_capture, ".bfbs")) {
      aos::WriteFlatbufferToFile(FLAGS_capture,
                                 image_fetcher->CopyFlatBuffer());
    } else {
      cv::imwrite(FLAGS_capture, bgr_image);
    }

    return false;
  }

  cv::Mat undistorted_image;
  cv::undistort(bgr_image, undistorted_image, intrinsics, dist_coeffs);

  if (april_debug.has_value() && april_debug.value()->corners()->size() > 0) {
    for (const auto *corners : *april_debug.value()->corners()) {
      std::vector<cv::Point> points;
      for (const auto *point_fbs : *corners->points()) {
        points.emplace_back(point_fbs->x(), point_fbs->y());
      }
      cv::polylines(undistorted_image, points, true, cv::Scalar(255, 0, 0), 10);
    }
  }

  cv::imshow("Display", undistorted_image);

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

  frc971::constants::WaitForConstants<Constants>(&config.message());

  aos::ShmEventLoop event_loop(&config.message());

  frc971::constants::ConstantsFetcher<Constants> constants_fetcher(&event_loop);
  const auto *calibration_data = FindCameraCalibration(
      constants_fetcher.constants(), event_loop.node()->name()->string_view());
  const cv::Mat intrinsics = CameraIntrinsics(calibration_data);
  const cv::Mat dist_coeffs = CameraDistCoeffs(calibration_data);

  aos::Fetcher<CameraImage> image_fetcher =
      event_loop.MakeFetcher<CameraImage>(FLAGS_channel);
  aos::Fetcher<AprilDebug> april_debug_fetcher =
      event_loop.MakeFetcher<AprilDebug>("/camera");

  // Run the display loop
  event_loop.AddPhasedLoop(
      [&](int) {
        if (!DisplayLoop(intrinsics, dist_coeffs, &image_fetcher,
                         &april_debug_fetcher)) {
          LOG(INFO) << "Calling event_loop Exit";
          event_loop.Exit();
        };
      },
      ::std::chrono::milliseconds(FLAGS_rate));

  event_loop.Run();

  image_fetcher = aos::Fetcher<CameraImage>();
}

}  // namespace
}  // namespace vision
}  // namespace y2023

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2023::vision::ViewerMain();
}
