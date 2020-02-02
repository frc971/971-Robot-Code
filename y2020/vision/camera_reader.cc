#include "aos/events/shm_event_loop.h"
#include "aos/init.h"

#include "y2020/vision/v4l2_reader.h"

namespace frc971 {
namespace vision {
namespace {

void CameraReaderMain() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  aos::ShmEventLoop event_loop(&config.message());
  V4L2Reader v4l2_reader(&event_loop, "/dev/video0");

  while (true) {
    const auto image = v4l2_reader.ReadLatestImage();
    if (image.empty()) {
      LOG(INFO) << "No image, sleeping";
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    // Now, process image.
    // TODO(Brian): Actually process it, rather than just logging its size...
    LOG(INFO) << image.size();
    std::this_thread::sleep_for(std::chrono::milliseconds(70));

    v4l2_reader.SendLatestImage();
  }
}

}  // namespace
}  // namespace vision
}  // namespace frc971


int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc971::vision::CameraReaderMain();
}
