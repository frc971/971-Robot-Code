#include "aos/vision/image/image_stream.h"

#include "aos/logging/logging.h"

namespace aos {
namespace vision {

void ImageStreamEvent::ProcessHelper(
    DataRef data, aos::monotonic_clock::time_point timestamp) {
  if (data.size() < 300) {
    AOS_LOG(INFO, "got bad img of size(%d)\n", static_cast<int>(data.size()));
    return;
  }
  ProcessImage(data, timestamp);
}

}  // namespace vision
}  // namespace aos
