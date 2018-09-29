#include <netdb.h>

#include "aos/logging/logging.h"
#include "aos/logging/queue_logging.h"
#include "aos/time/time.h"
#include "aos/linux_code/init.h"
#include "aos/vision/events/udp.h"
#include "y2018/vision.pb.h"
#include "y2018/vision/vision.q.h"

namespace y2018 {
namespace vision {

using aos::monotonic_clock;

int Main() {
  ::aos::events::RXUdpSocket video_rx(5001);
  char data[65507];
  ::y2018::VisionStatus status;

  while (true) {
    const ssize_t rx_size = video_rx.Recv(data, sizeof(data));
    if (rx_size > 0) {
      status.ParseFromArray(data, rx_size);
      auto new_vision_status = vision_status.MakeMessage();
      new_vision_status->high_frame_count = status.high_frame_count();
      new_vision_status->low_frame_count = status.low_frame_count();
      LOG_STRUCT(DEBUG, "vision", *new_vision_status);
      if (!new_vision_status.Send()) {
        LOG(ERROR, "Failed to send vision information\n");
      }
    }
  }
}

}  // namespace vision
}  // namespace y2018

int main(int /*argc*/, char ** /*argv*/) {
  ::aos::InitNRT();
  ::y2018::vision::Main();
}
