#include "aos/events/pong_lib.h"

#include "absl/flags/flag.h"
#include "absl/log/log.h"

#include "aos/events/event_loop.h"
#include "aos/events/ping_static.h"
#include "aos/events/pong_static.h"

ABSL_FLAG(bool, fetch, false,
          "Poll & fetch messages instead of using a watcher.");
ABSL_FLAG(uint32_t, fetch_period_ms, 10, "Frequency at which to fetch.");

namespace aos {

Pong::Pong(EventLoop *event_loop)
    : event_loop_(event_loop),
      fetcher_(event_loop_->MakeFetcher<examples::Ping>("/test")),
      sender_(event_loop_->MakeSender<examples::PongStatic>("/test")) {
  if (absl::GetFlag(FLAGS_fetch)) {
    event_loop_
        ->AddPhasedLoop(
            [this](int) {
              while (fetcher_.FetchNext()) {
                HandlePing(*fetcher_.get());
              }
            },
            std::chrono::milliseconds(absl::GetFlag(FLAGS_fetch_period_ms)))
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
  aos::Sender<examples::PongStatic>::StaticBuilder builder =
      sender_.MakeStaticBuilder();
  builder->set_value(ping.value());
  builder->set_initial_send_time(ping.send_time());
  builder.CheckOk(builder.Send());
}

}  // namespace aos
