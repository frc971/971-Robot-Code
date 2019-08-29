#include <chrono>

#include "aos/configuration.h"
#include "aos/events/pingpong_generated.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

namespace aos {

namespace chrono = std::chrono;

class Pong {
 public:
  Pong(EventLoop *event_loop)
      : event_loop_(event_loop),
        sender_(event_loop_->MakeSender<examples::Pong>("/test")) {
    event_loop_->MakeWatcher("/test", [this](const examples::Ping &ping) {
      aos::Sender<examples::Pong>::Builder msg = sender_.MakeBuilder();
      examples::Pong::Builder builder = msg.MakeBuilder<examples::Pong>();
      builder.add_value(ping.value());
      builder.add_initial_send_time(ping.send_time());
      CHECK(msg.Send(builder.Finish()));
    });

    event_loop_->SetRuntimeRealtimePriority(5);
  }

 private:
  EventLoop *event_loop_;
  aos::Sender<examples::Pong> sender_;
};

}  // namespace aos

int main(int argc, char **argv) {
  FLAGS_logtostderr = true;
  google::InitGoogleLogging(argv[0]);
  ::gflags::ParseCommandLineFlags(&argc, &argv, true);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos/events/config.fb.json");

  ::aos::ShmEventLoop event_loop(&config.message());

  aos::Pong ping(&event_loop);

  event_loop.Run();

  ::aos::Cleanup();
  return 0;
}
