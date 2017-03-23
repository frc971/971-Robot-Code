#include <netdb.h>

#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/time.h"
#include "aos/linux_code/init.h"
#include "aos/vision/events/udp.h"
#include "y2017/vision/target_finder.h"
#include "y2017/vision/vision.q.h"
#include "y2017/vision/vision_result.pb.h"

namespace y2017 {
namespace vision {

using aos::monotonic_clock;

int Main() {
  ::aos::events::RXUdpSocket recv(8080);
  char raw_data[65507];
  // TODO(parker): Have this pull in a config from somewhere.
  TargetFinder finder;

  while (true) {
    // TODO(austin): Don't malloc.
    VisionResult target;
    int size = recv.Recv(raw_data, sizeof(raw_data));
    monotonic_clock::time_point now = monotonic_clock::now();
    auto target_time = now -
                       std::chrono::nanoseconds(target.send_timestamp() -
                                                target.image_timestamp()) -
                       // It takes a bit to shoot a frame.  Push the frame
                       // further back in time.
                       std::chrono::milliseconds(60);

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

      finder.GetAngleDist(
          aos::vision::Vector<2>(target.target().x(), target.target().y()),
          /* TODO: Insert down estimate here in radians: */ 0.0,
          &new_vision_status->distance, &new_vision_status->angle);
    }

    LOG_STRUCT(DEBUG, "vision", *new_vision_status);
    if (!new_vision_status.Send()) {
      LOG(ERROR, "Failed to send vision information\n");
    }
  }
}

}  // namespace vision
}  // namespace y2017

int main(int /*argc*/, char ** /*argv*/) {
  ::aos::InitNRT();
  ::y2017::vision::Main();
}
