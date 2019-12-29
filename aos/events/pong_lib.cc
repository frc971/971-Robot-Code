#include "aos/events/pong_lib.h"

#include "aos/events/event_loop.h"
#include "aos/events/ping_generated.h"
#include "aos/events/pong_generated.h"
#include "glog/logging.h"

namespace aos {

Pong::Pong(EventLoop *event_loop)
    : event_loop_(event_loop),
      sender_(event_loop_->MakeSender<examples::Pong>("/test")) {
  event_loop_->MakeWatcher("/test", [this](const examples::Ping &ping) {
    aos::Sender<examples::Pong>::Builder builder = sender_.MakeBuilder();
    examples::Pong::Builder pong_builder =
        builder.MakeBuilder<examples::Pong>();
    pong_builder.add_value(ping.value());
    pong_builder.add_initial_send_time(ping.send_time());
    CHECK(builder.Send(pong_builder.Finish()));
  });

  event_loop_->SetRuntimeRealtimePriority(5);
}

}  // namespace aos
