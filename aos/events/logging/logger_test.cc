#include "aos/events/logging/logger.h"

#include "aos/events/event_loop.h"
#include "aos/events/ping_lib.h"
#include "aos/events/pong_lib.h"
#include "aos/events/simulated_event_loop.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
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

  // Even though it doesn't make any difference here, exercise the logic for
  // passing in a separate config.
  LogReader reader(logfile, &config_.message());

  // Confirm that we can remap logged channels to point to new buses.
  reader.RemapLoggedChannel<aos::examples::Ping>("/test", "/original");

  // This sends out the fetched messages and advances time to the start of the
  // log file.
  reader.Register();

  EXPECT_THAT(reader.Nodes(), ::testing::ElementsAre(nullptr));

  std::unique_ptr<EventLoop> test_event_loop =
      reader.event_loop_factory()->MakeEventLoop("log_reader");

  int ping_count = 10;
  int pong_count = 10;

  // Confirm that the ping value matches in the remapped channel location.
  test_event_loop->MakeWatcher("/original/test",
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

  reader.event_loop_factory()->RunFor(std::chrono::seconds(100));
  EXPECT_EQ(ping_count, 2010);
}

// Tests that we can read and write rotated log files.
TEST_F(LoggerTest, RotatedLogFile) {
  const ::std::string tmpdir(getenv("TEST_TMPDIR"));
  const ::std::string logfile0 = tmpdir + "/logfile0.bfbs";
  const ::std::string logfile1 = tmpdir + "/logfile1.bfbs";
  // Remove it.
  unlink(logfile0.c_str());
  unlink(logfile1.c_str());

  LOG(INFO) << "Logging data to " << logfile0 << " and " << logfile1;

  {
    DetachedBufferWriter writer0(logfile0);
    DetachedBufferWriter writer1(logfile1);
    std::unique_ptr<EventLoop> logger_event_loop =
        event_loop_factory_.MakeEventLoop("logger");

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    Logger logger(&writer0, logger_event_loop.get(),
                  std::chrono::milliseconds(100));
    event_loop_factory_.RunFor(chrono::milliseconds(10000));
    logger.Rotate(&writer1);
    event_loop_factory_.RunFor(chrono::milliseconds(10000));
  }

  // Even though it doesn't make any difference here, exercise the logic for
  // passing in a separate config.
  LogReader reader(std::vector<std::string>{logfile0, logfile1},
                   &config_.message());

  // Confirm that we can remap logged channels to point to new buses.
  reader.RemapLoggedChannel<aos::examples::Ping>("/test", "/original");

  // This sends out the fetched messages and advances time to the start of the
  // log file.
  reader.Register();

  EXPECT_THAT(reader.Nodes(), ::testing::ElementsAre(nullptr));

  std::unique_ptr<EventLoop> test_event_loop =
      reader.event_loop_factory()->MakeEventLoop("log_reader");

  int ping_count = 10;
  int pong_count = 10;

  // Confirm that the ping value matches in the remapped channel location.
  test_event_loop->MakeWatcher("/original/test",
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

  reader.event_loop_factory()->RunFor(std::chrono::seconds(100));
  EXPECT_EQ(ping_count, 2010);
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
        event_loop_factory_(&config_.message()),
        pi1_(
            configuration::GetNode(event_loop_factory_.configuration(), "pi1")),
        pi2_(
            configuration::GetNode(event_loop_factory_.configuration(), "pi2")),
        tmp_dir_(getenv("TEST_TMPDIR")),
        logfile_base_(tmp_dir_ + "/multi_logfile"),
        logfiles_({logfile_base_ + "_pi1_data.bfbs",
                   logfile_base_ + "_pi2_data/test/aos.examples.Pong.bfbs",
                   logfile_base_ + "_pi2_data.bfbs"}),
        ping_event_loop_(event_loop_factory_.MakeEventLoop("ping", pi1_)),
        ping_(ping_event_loop_.get()),
        pong_event_loop_(event_loop_factory_.MakeEventLoop("pong", pi2_)),
        pong_(pong_event_loop_.get()) {
    // Go through and remove the logfiles if they already exist.
    for (const auto file : logfiles_) {
      unlink(file.c_str());
    }

    LOG(INFO) << "Logging data to " << logfiles_[0] << ", " << logfiles_[1]
              << " and " << logfiles_[2];
  }

  struct LoggerState {
    std::unique_ptr<EventLoop> event_loop;
    std::unique_ptr<Logger> logger;
  };

  LoggerState MakeLogger(const Node *node) {
    return {event_loop_factory_.MakeEventLoop("logger", node), {}};
  }

  void StartLogger(LoggerState *logger) {
    logger->logger = std::make_unique<Logger>(
        std::make_unique<MultiNodeLogNamer>(logfile_base_,
                                            logger->event_loop->configuration(),
                                            logger->event_loop->node()),
        logger->event_loop.get(), chrono::milliseconds(100));
  }

  // Config and factory.
  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  SimulatedEventLoopFactory event_loop_factory_;

  const Node *pi1_;
  const Node *pi2_;

  std::string tmp_dir_;
  std::string logfile_base_;
  std::array<std::string, 3> logfiles_;

  std::unique_ptr<EventLoop> ping_event_loop_;
  Ping ping_;
  std::unique_ptr<EventLoop> pong_event_loop_;
  Pong pong_;
};


// Counts the number of messages on a channel (returns channel, count) for every
// message matching matcher()
std::vector<std::pair<int, int>> CountChannelsMatching(
    std::string_view filename,
    std::function<bool(const MessageHeader *)> matcher) {
  MessageReader message_reader(filename);
  std::vector<int> counts(
      message_reader.log_file_header()->configuration()->channels()->size(), 0);

  while (true) {
    std::optional<FlatbufferVector<MessageHeader>> msg =
        message_reader.ReadMessage();
    if (!msg) {
      break;
    }

    if (matcher(&msg.value().message())) {
      counts[msg.value().message().channel_index()]++;
    }
  }

  std::vector<std::pair<int, int>> result;
  int channel = 0;
  for (size_t i = 0; i < counts.size(); ++i) {
    if (counts[i] != 0) {
      result.push_back(std::make_pair(channel, counts[i]));
    }
    ++channel;
  }

  return result;
}

// Counts the number of messages (channel, count) for all data messages.
std::vector<std::pair<int, int>> CountChannelsData(std::string_view filename) {
  return CountChannelsMatching(filename, [](const MessageHeader *msg) {
    if (msg->has_data()) {
      CHECK(!msg->has_monotonic_remote_time());
      CHECK(!msg->has_realtime_remote_time());
      CHECK(!msg->has_remote_queue_index());
      return true;
    }
    return false;
  });
}

// Counts the number of messages (channel, count) for all timestamp messages.
std::vector<std::pair<int, int>> CountChannelsTimestamp(
    std::string_view filename) {
  return CountChannelsMatching(filename, [](const MessageHeader *msg) {
    if (!msg->has_data()) {
      CHECK(msg->has_monotonic_remote_time());
      CHECK(msg->has_realtime_remote_time());
      CHECK(msg->has_remote_queue_index());
      return true;
    }
    return false;
  });
}

// Tests that we can write and read simple multi-node log files.
TEST_F(MultinodeLoggerTest, SimpleMultiNode) {
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }

  {
    // Confirm that the headers are all for the correct nodes.
    FlatbufferVector<LogFileHeader> logheader1 = ReadHeader(logfiles_[0]);
    EXPECT_EQ(logheader1.message().node()->name()->string_view(), "pi1");
    FlatbufferVector<LogFileHeader> logheader2 = ReadHeader(logfiles_[1]);
    EXPECT_EQ(logheader2.message().node()->name()->string_view(), "pi2");
    FlatbufferVector<LogFileHeader> logheader3 = ReadHeader(logfiles_[2]);
    EXPECT_EQ(logheader3.message().node()->name()->string_view(), "pi2");

    // Timing reports, pings
    EXPECT_THAT(CountChannelsData(logfiles_[0]),
                ::testing::ElementsAre(::testing::Pair(1, 40),
                                       ::testing::Pair(4, 2001)));
    // Timestamps for pong
    EXPECT_THAT(CountChannelsTimestamp(logfiles_[0]),
                ::testing::ElementsAre(::testing::Pair(5, 2001)));

    // Pong data.
    EXPECT_THAT(CountChannelsData(logfiles_[1]),
                ::testing::ElementsAre(::testing::Pair(5, 2001)));
    // No timestamps
    EXPECT_THAT(CountChannelsTimestamp(logfiles_[1]), ::testing::ElementsAre());

    // Timing reports and pongs.
    EXPECT_THAT(CountChannelsData(logfiles_[2]),
                ::testing::ElementsAre(::testing::Pair(3, 40),
                                       ::testing::Pair(5, 2001)));
    // And ping timestamps.
    EXPECT_THAT(CountChannelsTimestamp(logfiles_[2]),
                ::testing::ElementsAre(::testing::Pair(4, 2001)));
  }

  LogReader reader({std::vector<std::string>{logfiles_[0]},
                    std::vector<std::string>{logfiles_[2]}});

  SimulatedEventLoopFactory log_reader_factory(reader.logged_configuration());
  log_reader_factory.set_send_delay(chrono::microseconds(0));

  // This sends out the fetched messages and advances time to the start of the
  // log file.
  reader.Register(&log_reader_factory);

  const Node *pi1 =
      configuration::GetNode(log_reader_factory.configuration(), "pi1");
  const Node *pi2 =
      configuration::GetNode(log_reader_factory.configuration(), "pi2");

  EXPECT_THAT(reader.Nodes(), ::testing::ElementsAre(pi1, pi2));

  reader.event_loop_factory()->set_send_delay(chrono::microseconds(0));

  std::unique_ptr<EventLoop> pi1_event_loop =
      log_reader_factory.MakeEventLoop("test", pi1);
  std::unique_ptr<EventLoop> pi2_event_loop =
      log_reader_factory.MakeEventLoop("test", pi2);

  int pi1_ping_count = 10;
  int pi2_ping_count = 10;
  int pi1_pong_count = 10;
  int pi2_pong_count = 10;

  // Confirm that the ping value matches.
  pi1_event_loop->MakeWatcher(
      "/test", [&pi1_ping_count, &pi1_event_loop](const examples::Ping &ping) {
        VLOG(1) << "Pi1 ping " << FlatbufferToJson(&ping)
                << pi1_event_loop->context().monotonic_remote_time << " -> "
                << pi1_event_loop->context().monotonic_event_time;
        EXPECT_EQ(ping.value(), pi1_ping_count + 1);
        EXPECT_EQ(pi1_event_loop->context().monotonic_remote_time,
                  pi1_ping_count * chrono::milliseconds(10) +
                      monotonic_clock::epoch());
        EXPECT_EQ(pi1_event_loop->context().realtime_remote_time,
                  pi1_ping_count * chrono::milliseconds(10) +
                      realtime_clock::epoch());
        EXPECT_EQ(pi1_event_loop->context().monotonic_remote_time,
                  pi1_event_loop->context().monotonic_event_time);
        EXPECT_EQ(pi1_event_loop->context().realtime_remote_time,
                  pi1_event_loop->context().realtime_event_time);

        ++pi1_ping_count;
      });
  pi2_event_loop->MakeWatcher(
      "/test", [&pi2_ping_count, &pi2_event_loop](const examples::Ping &ping) {
        VLOG(1) << "Pi2 ping " << FlatbufferToJson(&ping)
                << pi2_event_loop->context().monotonic_remote_time << " -> "
                << pi2_event_loop->context().monotonic_event_time;
        EXPECT_EQ(ping.value(), pi2_ping_count + 1);

        EXPECT_EQ(pi2_event_loop->context().monotonic_remote_time,
                  pi2_ping_count * chrono::milliseconds(10) +
                      monotonic_clock::epoch());
        EXPECT_EQ(pi2_event_loop->context().realtime_remote_time,
                  pi2_ping_count * chrono::milliseconds(10) +
                      realtime_clock::epoch());
        EXPECT_EQ(pi2_event_loop->context().monotonic_remote_time +
                      chrono::microseconds(150),
                  pi2_event_loop->context().monotonic_event_time);
        EXPECT_EQ(pi2_event_loop->context().realtime_remote_time +
                      chrono::microseconds(150),
                  pi2_event_loop->context().realtime_event_time);
        ++pi2_ping_count;
      });

  constexpr ssize_t kQueueIndexOffset = 0;
  // Confirm that the ping and pong counts both match, and the value also
  // matches.
  pi1_event_loop->MakeWatcher(
      "/test", [&pi1_event_loop, &pi1_ping_count,
                &pi1_pong_count](const examples::Pong &pong) {
        VLOG(1) << "Pi1 pong " << FlatbufferToJson(&pong) << " at "
                << pi1_event_loop->context().monotonic_remote_time << " -> "
                << pi1_event_loop->context().monotonic_event_time;

        EXPECT_EQ(pi1_event_loop->context().remote_queue_index,
                  pi1_pong_count + kQueueIndexOffset);
        EXPECT_EQ(pi1_event_loop->context().monotonic_remote_time,
                  chrono::microseconds(200) +
                      pi1_pong_count * chrono::milliseconds(10) +
                      monotonic_clock::epoch());
        EXPECT_EQ(pi1_event_loop->context().realtime_remote_time,
                  chrono::microseconds(200) +
                      pi1_pong_count * chrono::milliseconds(10) +
                      realtime_clock::epoch());

        EXPECT_EQ(pi1_event_loop->context().monotonic_remote_time +
                      chrono::microseconds(150),
                  pi1_event_loop->context().monotonic_event_time);
        EXPECT_EQ(pi1_event_loop->context().realtime_remote_time +
                      chrono::microseconds(150),
                  pi1_event_loop->context().realtime_event_time);

        EXPECT_EQ(pong.value(), pi1_pong_count + 1);
        ++pi1_pong_count;
        EXPECT_EQ(pi1_ping_count, pi1_pong_count);
      });
  pi2_event_loop->MakeWatcher(
      "/test", [&pi2_event_loop, &pi2_ping_count,
                &pi2_pong_count](const examples::Pong &pong) {
        VLOG(1) << "Pi2 pong " << FlatbufferToJson(&pong) << " at "
                << pi2_event_loop->context().monotonic_remote_time << " -> "
                << pi2_event_loop->context().monotonic_event_time;

        EXPECT_EQ(pi2_event_loop->context().remote_queue_index,
                  pi2_pong_count + kQueueIndexOffset - 9);

        EXPECT_EQ(pi2_event_loop->context().monotonic_remote_time,
                  chrono::microseconds(200) +
                      pi2_pong_count * chrono::milliseconds(10) +
                      monotonic_clock::epoch());
        EXPECT_EQ(pi2_event_loop->context().realtime_remote_time,
                  chrono::microseconds(200) +
                      pi2_pong_count * chrono::milliseconds(10) +
                      realtime_clock::epoch());

        EXPECT_EQ(pi2_event_loop->context().monotonic_remote_time,
                  pi2_event_loop->context().monotonic_event_time);
        EXPECT_EQ(pi2_event_loop->context().realtime_remote_time,
                  pi2_event_loop->context().realtime_event_time);

        EXPECT_EQ(pong.value(), pi2_pong_count + 1);
        ++pi2_pong_count;
        EXPECT_EQ(pi2_ping_count, pi2_pong_count);
      });

  log_reader_factory.Run();
  EXPECT_EQ(pi1_ping_count, 2010);
  EXPECT_EQ(pi2_ping_count, 2010);
  EXPECT_EQ(pi1_pong_count, 2010);
  EXPECT_EQ(pi2_pong_count, 2010);

  reader.Deregister();
}

typedef MultinodeLoggerTest MultinodeLoggerDeathTest;

// Test that if we feed the replay with a mismatched node list that we die on
// the LogReader constructor.
TEST_F(MultinodeLoggerDeathTest, MultiNodeBadReplayConfig) {
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }

  // Test that, if we add an additional node to the replay config that the
  // logger complains about the mismatch in number of nodes.
  FlatbufferDetachedBuffer<Configuration> extra_nodes_config =
      configuration::MergeWithConfig(&config_.message(), R"({
          "nodes": [
            {
              "name": "extra-node"
            }
          ]
        }
      )");

  EXPECT_DEATH(LogReader({std::vector<std::string>{logfiles_[0]},
                          std::vector<std::string>{logfiles_[2]}},
                         &extra_nodes_config.message()),
               "Log file and replay config need to have matching nodes lists.");
  ;
}

// Tests that we can read log files where they don't start at the same monotonic
// time.
TEST_F(MultinodeLoggerTest, StaggeredStart) {
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(200));

    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }

  LogReader reader({std::vector<std::string>{logfiles_[0]},
                    std::vector<std::string>{logfiles_[2]}});

  SimulatedEventLoopFactory log_reader_factory(reader.logged_configuration());
  log_reader_factory.set_send_delay(chrono::microseconds(0));

  // This sends out the fetched messages and advances time to the start of the
  // log file.
  reader.Register(&log_reader_factory);

  const Node *pi1 =
      configuration::GetNode(log_reader_factory.configuration(), "pi1");
  const Node *pi2 =
      configuration::GetNode(log_reader_factory.configuration(), "pi2");

  EXPECT_THAT(reader.Nodes(), ::testing::ElementsAre(pi1, pi2));

  reader.event_loop_factory()->set_send_delay(chrono::microseconds(0));

  std::unique_ptr<EventLoop> pi1_event_loop =
      log_reader_factory.MakeEventLoop("test", pi1);
  std::unique_ptr<EventLoop> pi2_event_loop =
      log_reader_factory.MakeEventLoop("test", pi2);

  int pi1_ping_count = 30;
  int pi2_ping_count = 30;
  int pi1_pong_count = 30;
  int pi2_pong_count = 30;

  // Confirm that the ping value matches.
  pi1_event_loop->MakeWatcher(
      "/test", [&pi1_ping_count, &pi1_event_loop](const examples::Ping &ping) {
        VLOG(1) << "Pi1 ping " << FlatbufferToJson(&ping)
                << pi1_event_loop->context().monotonic_remote_time << " -> "
                << pi1_event_loop->context().monotonic_event_time;
        EXPECT_EQ(ping.value(), pi1_ping_count + 1);

        ++pi1_ping_count;
      });
  pi2_event_loop->MakeWatcher(
      "/test", [&pi2_ping_count, &pi2_event_loop](const examples::Ping &ping) {
        VLOG(1) << "Pi2 ping " << FlatbufferToJson(&ping)
                << pi2_event_loop->context().monotonic_remote_time << " -> "
                << pi2_event_loop->context().monotonic_event_time;
        EXPECT_EQ(ping.value(), pi2_ping_count + 1);

        ++pi2_ping_count;
      });

  // Confirm that the ping and pong counts both match, and the value also
  // matches.
  pi1_event_loop->MakeWatcher(
      "/test", [&pi1_event_loop, &pi1_ping_count,
                &pi1_pong_count](const examples::Pong &pong) {
        VLOG(1) << "Pi1 pong " << FlatbufferToJson(&pong) << " at "
                << pi1_event_loop->context().monotonic_remote_time << " -> "
                << pi1_event_loop->context().monotonic_event_time;

        EXPECT_EQ(pong.value(), pi1_pong_count + 1);
        ++pi1_pong_count;
        EXPECT_EQ(pi1_ping_count, pi1_pong_count);
      });
  pi2_event_loop->MakeWatcher(
      "/test", [&pi2_event_loop, &pi2_ping_count,
                &pi2_pong_count](const examples::Pong &pong) {
        VLOG(1) << "Pi2 pong " << FlatbufferToJson(&pong) << " at "
                << pi2_event_loop->context().monotonic_remote_time << " -> "
                << pi2_event_loop->context().monotonic_event_time;

        EXPECT_EQ(pong.value(), pi2_pong_count + 1);
        ++pi2_pong_count;
        EXPECT_EQ(pi2_ping_count, pi2_pong_count);
      });

  log_reader_factory.Run();
  EXPECT_EQ(pi1_ping_count, 2030);
  EXPECT_EQ(pi2_ping_count, 2030);
  EXPECT_EQ(pi1_pong_count, 2030);
  EXPECT_EQ(pi2_pong_count, 2030);

  reader.Deregister();
}

// Tests that we can read log files where the monotonic clocks drift and don't
// match correctly.  While we are here, also test that different ending times
// also is readable.
TEST_F(MultinodeLoggerTest, MismatchedClocks) {
  {
    LoggerState pi2_logger = MakeLogger(pi2_);

    NodeEventLoopFactory *pi2 =
        event_loop_factory_.GetNodeEventLoopFactory(pi2_);
    LOG(INFO) << "pi2 times: " << pi2->monotonic_now() << " "
              << pi2->realtime_now() << " distributed "
              << pi2->ToDistributedClock(pi2->monotonic_now());

    const chrono::nanoseconds initial_pi2_offset = -chrono::seconds(1000);
    chrono::nanoseconds pi2_offset = initial_pi2_offset;

    pi2->SetDistributedOffset(pi2_offset);
    LOG(INFO) << "pi2 times: " << pi2->monotonic_now() << " "
              << pi2->realtime_now() << " distributed "
              << pi2->ToDistributedClock(pi2->monotonic_now());


    for (int i = 0; i < 95; ++i) {
      pi2_offset += chrono::nanoseconds(200);
      pi2->SetDistributedOffset(pi2_offset);
      event_loop_factory_.RunFor(chrono::milliseconds(1));
    }


    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(200));

    {
      // Run pi1's logger for only part of the time.
      LoggerState pi1_logger = MakeLogger(pi1_);

      StartLogger(&pi1_logger);

      for (int i = 0; i < 20000; ++i) {
        pi2_offset += chrono::nanoseconds(200);
        pi2->SetDistributedOffset(pi2_offset);
        event_loop_factory_.RunFor(chrono::milliseconds(1));
      }

      EXPECT_GT(pi2_offset - initial_pi2_offset,
                event_loop_factory_.send_delay() +
                    event_loop_factory_.network_delay());

      for (int i = 0; i < 40000; ++i) {
        pi2_offset -= chrono::nanoseconds(200);
        pi2->SetDistributedOffset(pi2_offset);
        event_loop_factory_.RunFor(chrono::milliseconds(1));
      }
    }

    // And log a bit more on pi2.
    event_loop_factory_.RunFor(chrono::milliseconds(400));
  }

  LogReader reader({std::vector<std::string>{logfiles_[0]},
                    std::vector<std::string>{logfiles_[2]}});

  SimulatedEventLoopFactory log_reader_factory(reader.logged_configuration());
  log_reader_factory.set_send_delay(chrono::microseconds(0));

  // This sends out the fetched messages and advances time to the start of the
  // log file.
  reader.Register(&log_reader_factory);


  const Node *pi1 =
      configuration::GetNode(log_reader_factory.configuration(), "pi1");
  const Node *pi2 =
      configuration::GetNode(log_reader_factory.configuration(), "pi2");

  LOG(INFO) << "Done registering (pi1) "
            << log_reader_factory.GetNodeEventLoopFactory(pi1)->monotonic_now() << " "
            << log_reader_factory.GetNodeEventLoopFactory(pi1)->realtime_now();
  LOG(INFO) << "Done registering (pi2) "
            << log_reader_factory.GetNodeEventLoopFactory(pi2)->monotonic_now() << " "
            << log_reader_factory.GetNodeEventLoopFactory(pi2)->realtime_now();

  EXPECT_THAT(reader.Nodes(), ::testing::ElementsAre(pi1, pi2));

  reader.event_loop_factory()->set_send_delay(chrono::microseconds(0));

  std::unique_ptr<EventLoop> pi1_event_loop =
      log_reader_factory.MakeEventLoop("test", pi1);
  std::unique_ptr<EventLoop> pi2_event_loop =
      log_reader_factory.MakeEventLoop("test", pi2);

  int pi1_ping_count = 30;
  int pi2_ping_count = 30;
  int pi1_pong_count = 30;
  int pi2_pong_count = 30;

  // Confirm that the ping value matches.
  pi1_event_loop->MakeWatcher(
      "/test", [&pi1_ping_count, &pi1_event_loop](const examples::Ping &ping) {
        VLOG(1) << "Pi1 ping " << FlatbufferToJson(&ping)
                << pi1_event_loop->context().monotonic_remote_time << " -> "
                << pi1_event_loop->context().monotonic_event_time;
        EXPECT_EQ(ping.value(), pi1_ping_count + 1);

        ++pi1_ping_count;
      });
  pi2_event_loop->MakeWatcher(
      "/test", [&pi2_ping_count, &pi2_event_loop](const examples::Ping &ping) {
        VLOG(1) << "Pi2 ping " << FlatbufferToJson(&ping)
                << pi2_event_loop->context().monotonic_remote_time << " -> "
                << pi2_event_loop->context().monotonic_event_time;
        EXPECT_EQ(ping.value(), pi2_ping_count + 1);

        ++pi2_ping_count;
      });

  // Confirm that the ping and pong counts both match, and the value also
  // matches.
  pi1_event_loop->MakeWatcher(
      "/test", [&pi1_event_loop, &pi1_ping_count,
                &pi1_pong_count](const examples::Pong &pong) {
        VLOG(1) << "Pi1 pong " << FlatbufferToJson(&pong) << " at "
                << pi1_event_loop->context().monotonic_remote_time << " -> "
                << pi1_event_loop->context().monotonic_event_time;

        EXPECT_EQ(pong.value(), pi1_pong_count + 1);
        ++pi1_pong_count;
        EXPECT_EQ(pi1_ping_count, pi1_pong_count);
      });
  pi2_event_loop->MakeWatcher(
      "/test", [&pi2_event_loop, &pi2_ping_count,
                &pi2_pong_count](const examples::Pong &pong) {
        VLOG(1) << "Pi2 pong " << FlatbufferToJson(&pong) << " at "
                << pi2_event_loop->context().monotonic_remote_time << " -> "
                << pi2_event_loop->context().monotonic_event_time;

        EXPECT_EQ(pong.value(), pi2_pong_count + 1);
        ++pi2_pong_count;
        EXPECT_EQ(pi2_ping_count, pi2_pong_count);
      });

  log_reader_factory.Run();
  EXPECT_EQ(pi1_ping_count, 6030);
  EXPECT_EQ(pi2_ping_count, 6030);
  EXPECT_EQ(pi1_pong_count, 6030);
  EXPECT_EQ(pi2_pong_count, 6030);

  reader.Deregister();
}

// TODO(austin): We can write a test which recreates a logfile and confirms that
// we get it back.  That is the ultimate test.

}  // namespace testing
}  // namespace logger
}  // namespace aos
