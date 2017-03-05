#include "aos/common/logging/implementations.h"
#include "aos/common/logging/logging.h"
#include "aos/vision/events/epoll_events.h"
#include "aos/vision/image/image_stream.h"

class ImageStream : public aos::vision::ImageStreamEvent {
 public:
  ImageStream(const std::string &fname, camera::CameraParams params)
      : ImageStreamEvent(fname, params) {}
  void ProcessImage(aos::vision::DataRef /*data*/,
                    aos::monotonic_clock::time_point) override {
    if (i_ > 20) {
      exit(0);
    }
    ++i_;
  }

 private:
  int i_ = 0;
};

// camera_primer drops the first 20 frames. This is to get around issues
// where the first N frames from the camera are garbage. Thus each year
// you should write a startup script like so:
//
// camera_primer
// target_sender
int main(int argc, char **argv) {
  ::aos::logging::Init();
  ::aos::logging::AddImplementation(
      new ::aos::logging::StreamLogImplementation(stdout));

  camera::CameraParams params = {.width = 640 * 2,
                                 .height = 480 * 2,
                                 .exposure = 10,
                                 .brightness = 128,
                                 .gain = 0,
                                 .fps = 30};

  if (argc != 2) {
    fprintf(stderr, "usage: %s path_to_camera\n", argv[0]);
    exit(-1);
  }
  ImageStream stream(argv[1], params);

  aos::events::EpollLoop loop;
  loop.Add(&stream);
  loop.Run();
}
