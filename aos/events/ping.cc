#include <chrono>

#include "aos/configuration.h"
#include "aos/events/pingpong_generated.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_int32(sleep_ms, 10, "Time to sleep between pings");

namespace aos {

namespace chrono = std::chrono;

class Ping {
 public:
  Ping(EventLoop *event_loop)
      : event_loop_(event_loop),
        sender_(event_loop_->MakeSender<examples::Ping>("/test")) {
    timer_handle_ = event_loop_->AddTimer([this]() { SendPing(); });

    event_loop_->MakeWatcher(
        "/test", [this](const examples::Pong &pong) { HandlePong(pong); });

    event_loop_->OnRun([this]() {
      timer_handle_->Setup(event_loop_->monotonic_now(),
                           chrono::milliseconds(FLAGS_sleep_ms));
    });

    event_loop_->SetRuntimeRealtimePriority(5);
  }

  void SendPing() {
    ++count_;
    aos::Sender<examples::Ping>::Builder msg = sender_.MakeBuilder();
    examples::Ping::Builder builder = msg.MakeBuilder<examples::Ping>();
    builder.add_value(count_);
    builder.add_send_time(
        event_loop_->monotonic_now().time_since_epoch().count());
    CHECK(msg.Send(builder.Finish()));
    VLOG(2) << "Sending ping";
  }

  void HandlePong(const examples::Pong &pong) {
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

 private:
  EventLoop *event_loop_;
  aos::Sender<examples::Ping> sender_;
  TimerHandler *timer_handle_;
  int count_ = 0;
};

}  // namespace aos

int main(int argc, char **argv) {
  FLAGS_logtostderr = true;
  google::InitGoogleLogging(argv[0]);
  ::gflags::ParseCommandLineFlags(&argc, &argv, true);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos/events/config.fb.json");

  ::aos::ShmEventLoop event_loop(&config.message());

  aos::Ping ping(&event_loop);

  event_loop.Run();

  ::aos::Cleanup();
  return 0;
}
