#include <netdb.h>

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/time/time.h"
#include "aos/vision/events/udp.h"
#include "y2018/vision.pb.h"
#include "y2018/vision/vision_generated.h"

namespace y2018 {
namespace vision {

using aos::monotonic_clock;

int Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::events::RXUdpSocket video_rx(5001);
  char data[65507];
  ::y2018::VisionStatus status;

  ::aos::ShmEventLoop event_loop(&config.message());
  ::aos::Sender<VisionStatus> vision_status_sender_ =
      event_loop.MakeSender<VisionStatus>("/superstructure");

  while (true) {
    const ssize_t rx_size = video_rx.Recv(data, sizeof(data));
    if (rx_size > 0) {
      status.ParseFromArray(data, rx_size);

      auto builder = vision_status_sender_.MakeBuilder();
      VisionStatus::Builder vision_status_builder =
          builder.MakeBuilder<VisionStatus>();
      vision_status_builder.add_high_frame_count(status.high_frame_count());
      vision_status_builder.add_low_frame_count(status.low_frame_count());
      if (!builder.Send(vision_status_builder.Finish())) {
        AOS_LOG(ERROR, "Failed to send vision information\n");
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
