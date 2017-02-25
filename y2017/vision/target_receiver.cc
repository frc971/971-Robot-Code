#include <netdb.h>

#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/time.h"
#include "aos/linux_code/init.h"
#include "aos/vision/events/udp.h"
#include "y2017/vision/vision.q.h"
#include "y2017/vision/vision_data.pb.h"

using aos::monotonic_clock;

namespace y2017 {
namespace vision {

void ComputeDistanceAngle(const Target &target, double *distance,
                          double *angle) {
  // TODO: fix this.
  *distance = target.y();
  *angle = target.x();
}

}  // namespace vision
}  // namespace y2017

int main() {
  using namespace y2017::vision;
  ::aos::events::RXUdpSocket recv(8080);
  char raw_data[65507];

  while (true) {
    // TODO(austin): Don't malloc.
    VisionData target;
    int size = recv.Recv(raw_data, sizeof(raw_data));
    monotonic_clock::time_point now = monotonic_clock::now();
    auto target_time = now -
                       std::chrono::nanoseconds(target.send_timestamp() -
                                                target.image_timestamp()) +
                       // It takes a bit to shoot a frame.  Push the frame
                       // further back in time.
                       std::chrono::milliseconds(10);

    if (!target.ParseFromArray(raw_data, size)) {
      continue;
    }

    auto new_vision_status = vision_status.MakeMessage();
    new_vision_status->image_valid = target.has_target();
    if (new_vision_status->image_valid) {
      new_vision_status->target_time =
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              target_time.time_since_epoch())
              .count();

      ComputeDistanceAngle(target.target(), &new_vision_status->distance,
                           &new_vision_status->angle);
    }

    LOG_STRUCT(DEBUG, "vision", *new_vision_status);
    if (!new_vision_status.Send()) {
      LOG(ERROR, "Failed to send vision information\n");
    }
  }
}
