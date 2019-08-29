#include <netdb.h>

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/time/time.h"
#include "aos/vision/events/udp.h"
#include "y2017/vision/target_finder.h"
#include "y2017/vision/vision_generated.h"
#include "y2017/vision/vision_result.pb.h"

namespace y2017 {
namespace vision {

using aos::monotonic_clock;

int Main() {
  ::aos::events::RXUdpSocket recv(8080);
  char raw_data[65507];
  // TODO(parker): Have this pull in a config from somewhere.
  TargetFinder finder;
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());

  ::aos::Sender<::y2017::vision::VisionStatus> vision_status_sender =
      event_loop.MakeSender<::y2017::vision::VisionStatus>("/vision");

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

    auto builder = vision_status_sender.MakeBuilder();
    VisionStatus::Builder vision_status_builder =
        builder.MakeBuilder<VisionStatus>();
    vision_status_builder.add_image_valid(target.has_target());
    if (target.has_target()) {
      vision_status_builder.add_target_time (
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              target_time.time_since_epoch())
              .count());

      double distance = 0.0;
      double angle = 0.0;
      finder.GetAngleDist(
          aos::vision::Vector<2>(target.target().x(), target.target().y()),
          /* TODO: Insert down estimate here in radians: */ 0.0,
          &distance, &angle);
      vision_status_builder.add_distance(distance);
      vision_status_builder.add_angle(angle);
    }

    if (!builder.Send(vision_status_builder.Finish())) {
      AOS_LOG(ERROR, "Failed to send vision information\n");
    }
  }
}

}  // namespace vision
}  // namespace y2017

int main(int /*argc*/, char ** /*argv*/) {
  ::aos::InitNRT();
  ::y2017::vision::Main();
}
