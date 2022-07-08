#include "aos/events/ping_lib.h"

#include "aos/events/ping_generated.h"
#include "aos/events/pong_generated.h"
#include "aos/json_to_flatbuffer.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_int32(sleep_ms, 10, "Time to sleep between pings");

namespace aos {

namespace chrono = std::chrono;

Ping::Ping(EventLoop *event_loop, std::string_view channel_name)
    : event_loop_(event_loop),
      sender_(event_loop_->MakeSender<examples::Ping>(channel_name)) {
  timer_handle_ = event_loop_->AddTimer([this]() { SendPing(); });
  timer_handle_->set_name("ping");

  if (event_loop_->HasChannel<examples::Pong>(channel_name)) {
    event_loop_->MakeWatcher(
        channel_name, [this](const examples::Pong &pong) { HandlePong(pong); });
  }

  event_loop_->OnRun([this]() {
    timer_handle_->Setup(event_loop_->monotonic_now(),
                         chrono::milliseconds(FLAGS_sleep_ms));
  });

  event_loop_->SetRuntimeRealtimePriority(5);
}

void Ping::SendPing() {
  if (last_pong_value_ != count_ && (!quiet_ || VLOG_IS_ON(1))) {
    LOG(WARNING) << "Did not receive response to " << count_ << " within "
                 << FLAGS_sleep_ms << "ms.";
  }
  ++count_;
  aos::Sender<examples::Ping>::Builder builder = sender_.MakeBuilder();
  examples::Ping::Builder ping_builder = builder.MakeBuilder<examples::Ping>();
  ping_builder.add_value(count_);
  ping_builder.add_send_time(
      event_loop_->monotonic_now().time_since_epoch().count());
  builder.CheckOk(builder.Send(ping_builder.Finish()));
  VLOG(2) << "Sending ping";
}

void Ping::HandlePong(const examples::Pong &pong) {
  const aos::monotonic_clock::time_point monotonic_send_time(
      chrono::nanoseconds(pong.initial_send_time()));
  const aos::monotonic_clock::time_point monotonic_now =
      event_loop_->monotonic_now();

  const chrono::nanoseconds round_trip_time =
      monotonic_now - monotonic_send_time;

  if (last_pong_value_ + 1 != pong.value() && (!quiet_ || VLOG_IS_ON(1))) {
    LOG(WARNING) << "Unexpected pong value, wanted " << last_pong_value_ + 1
                 << ", got " << pong.value();
  }

  if (pong.value() == count_) {
    VLOG(1) << "Elapsed time " << round_trip_time.count() << " ns "
            << FlatbufferToJson(&pong);
  } else if (!quiet_ || VLOG_IS_ON(1)) {
    LOG(WARNING) << "Unexpected pong response, got " << FlatbufferToJson(&pong)
                 << " expected " << count_ << ", elapsed time "
                 << round_trip_time.count() << " ns ";
  }

  last_pong_value_ = pong.value();
}

}  // namespace aos
