#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "aos/events/logging/logger.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "y2020/vision/vision_generated.h"

DEFINE_string(config, "y2020/config.json", "Path to the config file to use.");
DEFINE_string(logfile, "", "Path to the log file to use.");
DEFINE_string(node, "pi1", "Node name to replay.");

namespace frc971 {
namespace vision {
namespace {

void ViewerMain() {
  CHECK(!FLAGS_logfile.empty()) << "You forgot to specify a logfile.";

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::logger::LogReader reader(FLAGS_logfile, &config.message());
  reader.Register();
  const aos::Node *node = nullptr;
  if (aos::configuration::MultiNode(reader.configuration())) {
    node = aos::configuration::GetNode(reader.configuration(), FLAGS_node);
  }
  std::unique_ptr<aos::EventLoop> event_loop =
      reader.event_loop_factory()->MakeEventLoop("player", node);

  event_loop->MakeWatcher("/camera", [](const CameraImage &image) {
    cv::Mat image_mat(image.rows(), image.cols(), CV_8U);
    CHECK(image_mat.isContinuous());
    const int number_pixels = image.rows() * image.cols();
    for (int i = 0; i < number_pixels; ++i) {
      reinterpret_cast<uint8_t *>(image_mat.data)[i] =
          image.data()->data()[i * 2];
    }

    cv::imshow("Display", image_mat);
    cv::waitKey(1);
  });

  reader.event_loop_factory()->Run();
}

}  // namespace
}  // namespace vision
}  // namespace frc971

// Quick and lightweight grayscale viewer for images
int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc971::vision::ViewerMain();
}
