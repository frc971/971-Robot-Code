#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/time/time.h"
#include "frc971/vision/vision_generated.h"

DEFINE_string(config, "aos_config.json", "Path to the config file to use.");
DEFINE_string(channel, "/camera", "Channel name for the image.");

DEFINE_string(capture, "",
              "If set, capture a single image and save it to this filename.");

namespace frc971 {
namespace vision {
namespace {

aos::Fetcher<CameraImage> image_fetcher;
bool DisplayLoop() {
  const CameraImage *image;

  // Read next image
  if (!image_fetcher.Fetch()) {
    VLOG(2) << "Couldn't fetch next image";
    return true;
  }

  image = image_fetcher.get();
  CHECK(image != nullptr) << "Couldn't read image";

  // Create color image:
  cv::Mat image_color_mat(cv::Size(image->cols(), image->rows()), CV_8UC2,
                          (void *)image->data()->data());
  cv::Mat bgr_image(cv::Size(image->cols(), image->rows()), CV_8UC3);
  cv::cvtColor(image_color_mat, bgr_image, cv::COLOR_YUV2BGR_YUYV);

  if (!FLAGS_capture.empty()) {
    cv::imwrite(FLAGS_capture, bgr_image);
    return false;
  }

  cv::imshow("Display", bgr_image);
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

  image_fetcher = event_loop.MakeFetcher<CameraImage>(FLAGS_channel);

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

  image_fetcher = aos::Fetcher<CameraImage>();
}

}  // namespace
}  // namespace vision
}  // namespace frc971

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc971::vision::ViewerMain();
}
