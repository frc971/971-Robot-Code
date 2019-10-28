#include "y2019/jevois/camera/image_stream.h"

#include "aos/logging/logging.h"

namespace y2019 {
namespace camera {

void ImageStreamEvent::ProcessHelper(
    aos::vision::DataRef data, aos::monotonic_clock::time_point timestamp) {
  if (data.size() < 300) {
    AOS_LOG(INFO, "got bad img of size(%d)\n", static_cast<int>(data.size()));
    return;
  }
  ProcessImage(data, timestamp);
}

}  // namespace camera
}  // namespace y2019
