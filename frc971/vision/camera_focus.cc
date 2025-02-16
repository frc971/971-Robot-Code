#include "absl/flags/flag.h"
#include "absl/flags/usage.h"
#include "absl/log/log.h"
#include "opencv2/imgproc/imgproc.hpp"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/vision/vision_generated.h"

ABSL_FLAG(std::string, config, "aos_config.json", "Config file to use.");
ABSL_FLAG(std::string, camera_channel, "/camera0", "Camera channel to read");

int main(int argc, char *argv[]) {
  absl::SetProgramUsageMessage("");
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());

  event_loop.MakeWatcher(
      absl::GetFlag(FLAGS_camera_channel),
      [&](const frc971::vision::CameraImage &image) {
        cv::Mat image_mat(cv::Size(image.cols(), image.rows()), CV_8UC2,
                          (void *)image.data()->data());
        cv::Mat image_mat_color, image_mat_gray, laplace_mat;

        cv::cvtColor(image_mat, image_mat_color, cv::COLOR_YUV2BGR_YUYV);
        cv::cvtColor(image_mat_color, image_mat_gray, cv::COLOR_BGR2GRAY);
        cv::Laplacian(image_mat_gray, laplace_mat, CV_64F);
        cv::Scalar mean, stddev;
        cv::meanStdDev(laplace_mat, mean, stddev, cv::Mat());
        LOG(INFO) << "Focus(bigger is better): " << stddev * stddev;
      });

  event_loop.Run();
  return 0;
}
