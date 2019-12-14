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
  EXPECT_EQ(reader.node(), nullptr);

  SimulatedEventLoopFactory log_reader_factory(reader.configuration());

  // This sends out the fetched messages and advances time to the start of the
  // log file.
  reader.Register(&log_reader_factory);

  EXPECT_EQ(log_reader_factory.node(), nullptr);

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

class MultinodeLoggerTest : public ::testing::Test {
 public:
  MultinodeLoggerTest()
      : config_(aos::configuration::ReadConfig(
            "aos/events/logging/multinode_pingpong_config.json")),
        event_loop_factory_(&config_.message(), "pi1"),
        ping_event_loop_(event_loop_factory_.MakeEventLoop("ping")),
        ping_(ping_event_loop_.get()) {}

  // Config and factory.
  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  SimulatedEventLoopFactory event_loop_factory_;

  // Event loop and app for Ping
  std::unique_ptr<EventLoop> ping_event_loop_;
  Ping ping_;
};

// Tests that we can startup at all in a multinode configuration.
TEST_F(MultinodeLoggerTest, MultiNode) {
  constexpr chrono::seconds kTimeOffset = chrono::seconds(10000);
  constexpr uint32_t kQueueIndexOffset = 1024;
  const ::std::string tmpdir(getenv("TEST_TMPDIR"));
  const ::std::string logfile = tmpdir + "/multi_logfile.bfbs";
  // Remove it.
  unlink(logfile.c_str());

  LOG(INFO) << "Logging data to " << logfile;

  {
    std::unique_ptr<EventLoop> pong_event_loop =
        event_loop_factory_.MakeEventLoop("pong");

    std::unique_ptr<aos::RawSender> pong_sender(
        pong_event_loop->MakeRawSender(aos::configuration::GetChannel(
            pong_event_loop->configuration(), "/test", "aos.examples.Pong",
            pong_event_loop->name(), pong_event_loop->node())));

    // Ok, let's fake a remote node.  We use the fancy raw sender Send
    // method that message_gateway will use to do that.
    int pong_count = 0;
    pong_event_loop->MakeWatcher(
        "/test", [&pong_event_loop, &pong_count, &pong_sender,
                  kTimeOffset](const examples::Ping &ping) {
          flatbuffers::FlatBufferBuilder fbb;
          examples::Pong::Builder pong_builder(fbb);
          pong_builder.add_value(ping.value());
          pong_builder.add_initial_send_time(ping.send_time());
          fbb.Finish(pong_builder.Finish());

          pong_sender->Send(fbb.GetBufferPointer(), fbb.GetSize(),
                            pong_event_loop->monotonic_now() + kTimeOffset,
                            pong_event_loop->realtime_now() + kTimeOffset,
                            kQueueIndexOffset + pong_count);
          ++pong_count;
        });

    DetachedBufferWriter writer(logfile);
    std::unique_ptr<EventLoop> logger_event_loop =
        event_loop_factory_.MakeEventLoop("logger");

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    Logger logger(&writer, logger_event_loop.get(),
                  std::chrono::milliseconds(100));
    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }

  LogReader reader(logfile);
  ASSERT_NE(reader.node(), nullptr);
  EXPECT_EQ(reader.node()->name()->string_view(), "pi1");

  // TODO(austin): Also replay as pi2 or pi3 and make sure we see the pong
  // messages.  This won't work today yet until the log reading code gets
  // significantly better.
  SimulatedEventLoopFactory log_reader_factory(reader.configuration(),
                                               reader.node());
  log_reader_factory.set_send_delay(chrono::microseconds(0));

  // This sends out the fetched messages and advances time to the start of the
  // log file.
  reader.Register(&log_reader_factory);

  std::unique_ptr<EventLoop> test_event_loop =
      log_reader_factory.MakeEventLoop("test");

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
      "/test", [&test_event_loop, &ping_count, &pong_count,
                kTimeOffset](const examples::Pong &pong) {
        EXPECT_EQ(test_event_loop->context().remote_queue_index,
                  pong_count + kQueueIndexOffset);
        EXPECT_EQ(test_event_loop->context().monotonic_remote_time,
                  test_event_loop->monotonic_now() + kTimeOffset);
        EXPECT_EQ(test_event_loop->context().realtime_remote_time,
                  test_event_loop->realtime_now() + kTimeOffset);

        EXPECT_EQ(pong.value(), pong_count + 1);
        ++pong_count;
        EXPECT_EQ(ping_count, pong_count);
      });

  log_reader_factory.RunFor(std::chrono::seconds(100));
  EXPECT_EQ(ping_count, 2010);
  EXPECT_EQ(pong_count, 2010);

  reader.Deregister();
}

}  // namespace testing
}  // namespace logger
}  // namespace aos
