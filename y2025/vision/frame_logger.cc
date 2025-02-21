#include "frame_logger.h"

#include "absl/flags/flag.h"
#include "absl/log/log.h"

#include "aos/time/time.h"

ABSL_FLAG(bool, camera_zero, true, "If we are logging on camera zero");
ABSL_FLAG(bool, camera_one, true, "If we are logging on camera one");

namespace y2025::vision {

using aos::monotonic_clock;

FrameLogger::FrameLogger(aos::EventLoop *event_loop) {
  auto logImage = [this, event_loop](std::string path) {
    return [this, event_loop, path](const CameraImage &image) {
      const monotonic_clock::time_point eof = monotonic_clock::time_point(
          std::chrono::nanoseconds(image.monotonic_timestamp_ns()));
      const monotonic_clock::duration age = event_loop->monotonic_now() - eof;
      if (age > std::chrono::milliseconds(100)) {
        VLOG(1) << "Behind, skipping image on camera0";
        return;
      }
      LogImage(path, &image);
    };
  };
  if (absl::GetFlag(FLAGS_camera_zero)) {
    event_loop->MakeWatcher("/camera0", logImage("camera0"));
  }
  if (absl::GetFlag(FLAGS_camera_one)) {
    event_loop->MakeWatcher("/camera1", logImage("camera1"));
  }
}

void FrameLogger::LogImage(const std::string channel_name,
                           const CameraImage *image) {
  cv::Mat image_color_mat(cv::Size(image->cols(), image->rows()), CV_8UC2,
                          (void *)image->data()->data());
  cv::Mat image_mat;

  cv::cvtColor(image_color_mat, image_mat, cv::COLOR_YUV2BGR_YUYV);

  std::vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(0);

  const std::string filepath = "/home/pi/images/" + channel_name + "-" +
                               std::to_string(image->monotonic_timestamp_ns()) +
                               ".png";

  cv::imwrite(filepath, image_mat, compression_params);
  VLOG(0) << "Added Image: " << filepath;
}

}  // namespace y2025::vision
