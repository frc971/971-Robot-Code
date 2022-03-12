#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/flatbuffers.h"
#include "frc971/vision/vision_generated.h"

DEFINE_string(config, "aos_config.json", "Path to the config file to use.");

namespace frc971::vision {
// Reads images from /camera and resends them in /camera/decimated at a fixed
// rate (1 Hz, in this case).
class ImageDecimator {
 public:
  ImageDecimator(aos::EventLoop *event_loop)
      : slow_image_sender_(
            event_loop->MakeSender<CameraImage>("/camera/decimated")),
        image_fetcher_(event_loop->MakeFetcher<CameraImage>("/camera")) {
    aos::TimerHandler *timer =
        event_loop->AddTimer(
            [this]() {
              if (image_fetcher_.Fetch()) {
                const aos::FlatbufferSpan<CameraImage> image(
                    {reinterpret_cast<const uint8_t *>(
                         image_fetcher_.context().data),
                     image_fetcher_.context().size});
                slow_image_sender_.CheckOk(slow_image_sender_.Send(image));
              }
            });
    event_loop->OnRun([event_loop, timer]() {
      timer->Setup(event_loop->monotonic_now(),
                   std::chrono::milliseconds(1000));
    });
  }

 private:
  aos::Sender<CameraImage> slow_image_sender_;
  aos::Fetcher<CameraImage> image_fetcher_;
};
}

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());
  frc971::vision::ImageDecimator decimator(&event_loop);

  event_loop.Run();

  return 0;
}
