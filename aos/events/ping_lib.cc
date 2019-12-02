#include "aos/events/ping_lib.h"

#include "aos/events/pong_generated.h"
#include "aos/events/ping_generated.h"
#include "aos/json_to_flatbuffer.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_int32(sleep_ms, 10, "Time to sleep between pings");
DEFINE_bool(phased_loop, false, "If true, use a phased loop");

namespace aos {

namespace chrono = std::chrono;

Ping::Ping(EventLoop *event_loop)
    : event_loop_(event_loop),
      sender_(event_loop_->MakeSender<examples::Ping>("/test")),
      pong_fetcher_(event_loop_->MakeFetcher<examples::Pong>("/test")) {
  if (FLAGS_phased_loop) {
    phased_loop_handle_ = event_loop_->AddPhasedLoop(
        [this](int) { SendPing(); }, chrono::milliseconds(FLAGS_sleep_ms));
    phased_loop_handle_->set_name("ping");
  } else {
    timer_handle_ = event_loop_->AddTimer([this]() { SendPing(); });
    timer_handle_->set_name("ping");
  }

  event_loop_->MakeWatcher(
      "/test", [this](const examples::Pong &pong) { HandlePong(pong); });

  event_loop_->OnRun([this]() {
    if (!FLAGS_phased_loop) {
      timer_handle_->Setup(event_loop_->monotonic_now(),
                           chrono::milliseconds(FLAGS_sleep_ms));
    }
  });

  event_loop_->SetRuntimeRealtimePriority(5);
}

void Ping::SendPing() {
  ++count_;
  aos::Sender<examples::Ping>::Builder msg = sender_.MakeBuilder();
  examples::Ping::Builder builder = msg.MakeBuilder<examples::Ping>();
  builder.add_value(count_);
  builder.add_send_time(
      event_loop_->monotonic_now().time_since_epoch().count());
  CHECK(msg.Send(builder.Finish()));
  VLOG(2) << "Sending ping";
}

void Ping::HandlePong(const examples::Pong &pong) {
  pong_fetcher_.Fetch();
  const aos::monotonic_clock::time_point monotonic_send_time(
      chrono::nanoseconds(pong.initial_send_time()));
  const aos::monotonic_clock::time_point monotonic_now =
      event_loop_->monotonic_now();

  const chrono::nanoseconds round_trip_time =
      monotonic_now - monotonic_send_time;

  if (pong.value() == count_) {
    VLOG(1) << "Elapsed time " << round_trip_time.count() << " ns "
            << FlatbufferToJson(&pong);
  } else {
    VLOG(1) << "Missmatched pong message";
  }
}
}  // namespace aos
