#include <sys/stat.h>

#include "aos/events/event_loop.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/logging/dynamic_logging.h"
#include "aos/testing/path.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

using aos::testing::ArtifactPath;

namespace aos {
namespace logging {
namespace testing {

namespace chrono = std::chrono;

class DynamicLoggingTest : public ::testing::Test {
 public:
  DynamicLoggingTest()
      : config_(aos::configuration::ReadConfig(
            ArtifactPath("aos/events/pingpong_config.json"))),
        event_loop_factory_(&config_.message()),
        event_loop_send_(event_loop_factory_.MakeEventLoop("send")),
        event_loop_main_(event_loop_factory_.MakeEventLoop("main")) {}

  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  SimulatedEventLoopFactory event_loop_factory_;

  std::unique_ptr<EventLoop> event_loop_send_;
  std::unique_ptr<EventLoop> event_loop_main_;
};

TEST_F(DynamicLoggingTest, TestVLog) {
  aos::Sender<DynamicLogCommand> dynamic_log_command_sender =
      event_loop_send_->MakeSender<DynamicLogCommand>("/aos");

  // Set VLOG level to 1 at t=50us and then back to 0 at t=150us.
  int log_level = 1;
  aos::TimerHandler *timer_handler = event_loop_send_->AddTimer(
      [this, &dynamic_log_command_sender, &log_level, &timer_handler]() {
        aos::Sender<DynamicLogCommand>::Builder message =
            dynamic_log_command_sender.MakeBuilder();
        const auto name_str = message.fbb()->CreateString("main");

        DynamicLogCommand::Builder builder =
            message.MakeBuilder<DynamicLogCommand>();
        builder.add_name(name_str);
        builder.add_vlog_level(log_level);
        CHECK_EQ(message.Send(builder.Finish()), RawSender::Error::kOk);
        --log_level;
        if (log_level >= 0) {
          timer_handler->Setup(event_loop_send_->monotonic_now() +
                               chrono::microseconds(100));
        }
      });
  timer_handler->Setup(event_loop_send_->monotonic_now() +
                       chrono::microseconds(50));

  // VLOG(1) at t=0us, t=100us, t=200us
  aos::TimerHandler *vlog_timer_handler =
      event_loop_main_->AddTimer([]() { VLOG(1) << "VLOG 1"; });
  vlog_timer_handler->Setup(event_loop_main_->monotonic_now(),
                            chrono::microseconds(100));

  DynamicLogging dynamic_logging(event_loop_main_.get());

  {
    // Validate no log message in first 50us.
    ::testing::internal::CaptureStderr();
    event_loop_factory_.RunFor(chrono::microseconds(50));
    std::string output = ::testing::internal::GetCapturedStderr();
    EXPECT_EQ(output.find("VLOG 1"), std::string::npos);
  }
  {
    // Validate 1 log message between 50us to 150us
    ::testing::internal::CaptureStderr();
    event_loop_factory_.RunFor(chrono::microseconds(100));
    std::string output = ::testing::internal::GetCapturedStderr();
    EXPECT_NE(output.find("VLOG 1"), std::string::npos);
  }
  {
    // Validate no log message between 150us to 250us
    ::testing::internal::CaptureStderr();
    event_loop_factory_.RunFor(chrono::microseconds(100));
    std::string output = ::testing::internal::GetCapturedStderr();
    EXPECT_EQ(output.find("VLOG 1"), std::string::npos);
  }
}

}  // namespace testing
}  // namespace logging
}  // namespace aos
