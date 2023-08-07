#include "aos/events/pong_lib.h"

#include "glog/logging.h"

#include "aos/events/event_loop.h"
#include "aos/events/ping_generated.h"
#include "aos/events/pong_generated.h"

DEFINE_bool(fetch, false, "Poll & fetch messages instead of using a watcher.");
DEFINE_uint32(fetch_period_ms, 10, "Frequency at which to fetch.");

namespace aos {

Pong::Pong(EventLoop *event_loop)
    : event_loop_(event_loop),
      fetcher_(event_loop_->MakeFetcher<examples::Ping>("/test")),
      sender_(event_loop_->MakeSender<examples::Pong>("/test")) {
  if (FLAGS_fetch) {
    event_loop_
        ->AddPhasedLoop(
            [this](int) {
              while (fetcher_.FetchNext()) {
                HandlePing(*fetcher_.get());
              }
            },
            std::chrono::milliseconds(FLAGS_fetch_period_ms))
        ->set_name("pong");
  } else {
    event_loop_->MakeWatcher(
        "/test", [this](const examples::Ping &ping) { HandlePing(ping); });
  }

  event_loop_->SetRuntimeRealtimePriority(5);
}

void Pong::HandlePing(const examples::Ping &ping) {
  if (last_value_ == ping.value() && (!quiet_ || VLOG_IS_ON(1))) {
    LOG(WARNING) << "Duplicate ping value at " << last_value_
                 << " time difference " << ping.send_time() - last_send_time_;
  }
  last_value_ = ping.value();
  last_send_time_ = ping.send_time();
  aos::Sender<examples::Pong>::Builder builder = sender_.MakeBuilder();
  examples::Pong::Builder pong_builder = builder.MakeBuilder<examples::Pong>();
  pong_builder.add_value(ping.value());
  pong_builder.add_initial_send_time(ping.send_time());
  builder.CheckOk(builder.Send(pong_builder.Finish()));
}

}  // namespace aos
