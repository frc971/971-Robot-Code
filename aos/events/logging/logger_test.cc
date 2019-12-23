#include "aos/events/logging/logger.h"

#include "aos/events/event_loop.h"
#include "aos/events/ping_lib.h"
#include "aos/events/pong_lib.h"
#include "aos/events/simulated_event_loop.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace aos {
namespace logger {
namespace testing {

namespace chrono = std::chrono;

class LoggerTest : public ::testing::Test {
 public:
  LoggerTest()
      : config_(
            aos::configuration::ReadConfig("aos/events/pingpong_config.json")),
        event_loop_factory_(&config_.message()),
        ping_event_loop_(event_loop_factory_.MakeEventLoop("ping")),
        ping_(ping_event_loop_.get()),
        pong_event_loop_(event_loop_factory_.MakeEventLoop("pong")),
        pong_(pong_event_loop_.get()) {}

  // Config and factory.
  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  SimulatedEventLoopFactory event_loop_factory_;

  // Event loop and app for Ping
  std::unique_ptr<EventLoop> ping_event_loop_;
  Ping ping_;

  // Event loop and app for Pong
  std::unique_ptr<EventLoop> pong_event_loop_;
  Pong pong_;
};

// Tests that we can startup at all.  This confirms that the channels are all in
// the config.
TEST_F(LoggerTest, Starts) {
  const ::std::string tmpdir(getenv("TEST_TMPDIR"));
  const ::std::string logfile = tmpdir + "/logfile.bfbs";
  // Remove it.
  unlink(logfile.c_str());

  LOG(INFO) << "Logging data to " << logfile;

  {
    DetachedBufferWriter writer(logfile);
    std::unique_ptr<EventLoop> logger_event_loop =
        event_loop_factory_.MakeEventLoop("logger");

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    Logger logger(&writer, logger_event_loop.get(),
                  std::chrono::milliseconds(100));
    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }

  LogReader reader(logfile);

  LOG(INFO) << "Config " << FlatbufferToJson(reader.configuration());

  // TODO(austin): Figure out what the API needs to look like.  How do we replay
  // the data that was fetched before it all starts?  How do we set the starting
  // time from the log file?  Probably need to let the reader do more if it
  // knows about the factory.
  SimulatedEventLoopFactory log_reader_factory(reader.configuration());
  std::unique_ptr<EventLoop> reader_event_loop =
      log_reader_factory.MakeEventLoop("log_reader");

  reader.Register(reader_event_loop.get());

  // Capture monotonic start time in OnRun and offset from there?  Let the user
  // configure the factory if they want time to match?
  /*
  log_reader_factory.InitializeTime(log_reader.monotonic_start_time(),
                                    log_reader.realtime_start_time());
                                    */
  std::unique_ptr<EventLoop> test_event_loop =
      log_reader_factory.MakeEventLoop("log_reader");

  int ping_count = 10;
  int pong_count = 10;

  // Confirm that the ping value matches.
  test_event_loop->MakeWatcher("/test",
                               [&ping_count](const examples::Ping &ping) {
                                 EXPECT_EQ(ping.value(), ping_count + 1);
                                 ++ping_count;
                               });
  // Confirm that the ping and pong counts both match, and the value also
  // matches.
  test_event_loop->MakeWatcher(
      "/test", [&pong_count, &ping_count](const examples::Pong &pong) {
        EXPECT_EQ(pong.value(), pong_count + 1);
        ++pong_count;
        EXPECT_EQ(ping_count, pong_count);
      });

  log_reader_factory.RunFor(std::chrono::seconds(100));
  EXPECT_EQ(ping_count, 2010);

  reader.Deregister();
}

// Tests that a large number of messages per second doesn't overwhelm writev.
TEST_F(LoggerTest, ManyMessages) {
  const ::std::string tmpdir(getenv("TEST_TMPDIR"));
  const ::std::string logfile = tmpdir + "/logfile.bfbs";
  // Remove the log file.
  unlink(logfile.c_str());

  LOG(INFO) << "Logging data to " << logfile;

  {
    DetachedBufferWriter writer(logfile);
    std::unique_ptr<EventLoop> logger_event_loop =
        event_loop_factory_.MakeEventLoop("logger");

    std::unique_ptr<EventLoop> ping_spammer_event_loop =
        event_loop_factory_.MakeEventLoop("ping_spammer");
    aos::Sender<examples::Ping> ping_sender =
        ping_spammer_event_loop->MakeSender<examples::Ping>("/test");

    aos::TimerHandler *timer_handler =
        ping_spammer_event_loop->AddTimer([&ping_sender]() {
          aos::Sender<examples::Ping>::Builder builder =
              ping_sender.MakeBuilder();
          examples::Ping::Builder ping_builder =
              builder.MakeBuilder<examples::Ping>();
          CHECK(builder.Send(ping_builder.Finish()));
        });

    // 100 ms / 0.05 ms -> 2000 messages.  Should be enough to crash it.
    ping_spammer_event_loop->OnRun([&ping_spammer_event_loop, timer_handler]() {
      timer_handler->Setup(ping_spammer_event_loop->monotonic_now(),
                           chrono::microseconds(50));
    });

    Logger logger(&writer, logger_event_loop.get(),
                  std::chrono::milliseconds(100));

    event_loop_factory_.RunFor(chrono::milliseconds(1000));
  }
}

}  // namespace testing
}  // namespace logger
}  // namespace aos
