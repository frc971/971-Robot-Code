#include "aos/events/logging/logger.h"

#include "absl/strings/str_format.h"
#include "aos/events/event_loop.h"
#include "aos/events/message_counter.h"
#include "aos/events/ping_lib.h"
#include "aos/events/pong_lib.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/remote_message_generated.h"
#include "aos/network/timestamp_generated.h"
#include "aos/testing/tmpdir.h"
#include "aos/util/file.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#ifdef LZMA
#include "aos/events/logging/lzma_encoder.h"
#endif

namespace aos {
namespace logger {
namespace testing {

namespace chrono = std::chrono;
using aos::message_bridge::RemoteMessage;
using aos::testing::MessageCounter;

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

using LoggerDeathTest = LoggerTest;

// Tests that we can startup at all.  This confirms that the channels are all in
// the config.
TEST_F(LoggerTest, Starts) {
  const ::std::string tmpdir = aos::testing::TestTmpDir();
  const ::std::string base_name = tmpdir + "/logfile";
  const ::std::string logfile = base_name + ".part0.bfbs";
  // Remove it.
  unlink(logfile.c_str());

  LOG(INFO) << "Logging data to " << logfile;

  {
    std::unique_ptr<EventLoop> logger_event_loop =
        event_loop_factory_.MakeEventLoop("logger");

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    Logger logger(logger_event_loop.get());
    logger.set_polling_period(std::chrono::milliseconds(100));
    logger.StartLoggingLocalNamerOnRun(base_name);
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

// Tests calling StartLogging twice.
TEST_F(LoggerDeathTest, ExtraStart) {
  const ::std::string tmpdir = aos::testing::TestTmpDir();
  const ::std::string base_name1 = tmpdir + "/logfile1";
  const ::std::string logfile1 = base_name1 + ".part0.bfbs";
  const ::std::string base_name2 = tmpdir + "/logfile2";
  const ::std::string logfile2 = base_name2 + ".part0.bfbs";
  unlink(logfile1.c_str());
  unlink(logfile2.c_str());

  LOG(INFO) << "Logging data to " << logfile1 << " then " << logfile2;

  {
    std::unique_ptr<EventLoop> logger_event_loop =
        event_loop_factory_.MakeEventLoop("logger");

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    Logger logger(logger_event_loop.get());
    logger.set_polling_period(std::chrono::milliseconds(100));
    logger_event_loop->OnRun(
        [base_name1, base_name2, &logger_event_loop, &logger]() {
          logger.StartLogging(std::make_unique<LocalLogNamer>(
              base_name1, logger_event_loop->node()));
          EXPECT_DEATH(logger.StartLogging(std::make_unique<LocalLogNamer>(
                           base_name2, logger_event_loop->node())),
                       "Already logging");
        });
    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }
}

// Tests calling StopLogging twice.
TEST_F(LoggerDeathTest, ExtraStop) {
  const ::std::string tmpdir = aos::testing::TestTmpDir();
  const ::std::string base_name = tmpdir + "/logfile";
  const ::std::string logfile = base_name + ".part0.bfbs";
  // Remove it.
  unlink(logfile.c_str());

  LOG(INFO) << "Logging data to " << logfile;

  {
    std::unique_ptr<EventLoop> logger_event_loop =
        event_loop_factory_.MakeEventLoop("logger");

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    Logger logger(logger_event_loop.get());
    logger.set_polling_period(std::chrono::milliseconds(100));
    logger_event_loop->OnRun([base_name, &logger_event_loop, &logger]() {
      logger.StartLogging(std::make_unique<LocalLogNamer>(
          base_name, logger_event_loop->node()));
      logger.StopLogging(aos::monotonic_clock::min_time);
      EXPECT_DEATH(logger.StopLogging(aos::monotonic_clock::min_time),
                   "Not logging right now");
    });
    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }
}

// Tests that we can startup twice.
TEST_F(LoggerTest, StartsTwice) {
  const ::std::string tmpdir = aos::testing::TestTmpDir();
  const ::std::string base_name1 = tmpdir + "/logfile1";
  const ::std::string logfile1 = base_name1 + ".part0.bfbs";
  const ::std::string base_name2 = tmpdir + "/logfile2";
  const ::std::string logfile2 = base_name2 + ".part0.bfbs";
  unlink(logfile1.c_str());
  unlink(logfile2.c_str());

  LOG(INFO) << "Logging data to " << logfile1 << " then " << logfile2;

  {
    std::unique_ptr<EventLoop> logger_event_loop =
        event_loop_factory_.MakeEventLoop("logger");

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    Logger logger(logger_event_loop.get());
    logger.set_polling_period(std::chrono::milliseconds(100));
    logger.StartLogging(
        std::make_unique<LocalLogNamer>(base_name1, logger_event_loop->node()));
    event_loop_factory_.RunFor(chrono::milliseconds(10000));
    logger.StopLogging(logger_event_loop->monotonic_now());
    event_loop_factory_.RunFor(chrono::milliseconds(10000));
    logger.StartLogging(
        std::make_unique<LocalLogNamer>(base_name2, logger_event_loop->node()));
    event_loop_factory_.RunFor(chrono::milliseconds(10000));
  }

  for (const auto &logfile :
       {std::make_tuple(logfile1, 10), std::make_tuple(logfile2, 2010)}) {
    SCOPED_TRACE(std::get<0>(logfile));
    LogReader reader(std::get<0>(logfile));
    reader.Register();

    EXPECT_THAT(reader.Nodes(), ::testing::ElementsAre(nullptr));

    std::unique_ptr<EventLoop> test_event_loop =
        reader.event_loop_factory()->MakeEventLoop("log_reader");

    int ping_count = std::get<1>(logfile);
    int pong_count = std::get<1>(logfile);

    // Confirm that the ping and pong counts both match, and the value also
    // matches.
    test_event_loop->MakeWatcher("/test",
                                 [&ping_count](const examples::Ping &ping) {
                                   EXPECT_EQ(ping.value(), ping_count + 1);
                                   ++ping_count;
                                 });
    test_event_loop->MakeWatcher(
        "/test", [&pong_count, &ping_count](const examples::Pong &pong) {
          EXPECT_EQ(pong.value(), pong_count + 1);
          ++pong_count;
          EXPECT_EQ(ping_count, pong_count);
        });

    reader.event_loop_factory()->RunFor(std::chrono::seconds(100));
    EXPECT_EQ(ping_count, std::get<1>(logfile) + 1000);
  }
}

// Tests that we can read and write rotated log files.
TEST_F(LoggerTest, RotatedLogFile) {
  const ::std::string tmpdir = aos::testing::TestTmpDir();
  const ::std::string base_name = tmpdir + "/logfile";
  const ::std::string logfile0 = base_name + ".part0.bfbs";
  const ::std::string logfile1 = base_name + ".part1.bfbs";
  // Remove it.
  unlink(logfile0.c_str());
  unlink(logfile1.c_str());

  LOG(INFO) << "Logging data to " << logfile0 << " and " << logfile1;

  {
    std::unique_ptr<EventLoop> logger_event_loop =
        event_loop_factory_.MakeEventLoop("logger");

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    Logger logger(logger_event_loop.get());
    logger.set_polling_period(std::chrono::milliseconds(100));
    logger.StartLoggingLocalNamerOnRun(base_name);
    event_loop_factory_.RunFor(chrono::milliseconds(10000));
    logger.Rotate();
    event_loop_factory_.RunFor(chrono::milliseconds(10000));
  }

  {
    // Confirm that the UUIDs match for both the parts and the logger, and the
    // parts_index increments.
    std::vector<SizePrefixedFlatbufferVector<LogFileHeader>> log_header;
    for (std::string_view f : {logfile0, logfile1}) {
      log_header.emplace_back(ReadHeader(f).value());
    }

    EXPECT_EQ(log_header[0].message().log_event_uuid()->string_view(),
              log_header[1].message().log_event_uuid()->string_view());
    EXPECT_EQ(log_header[0].message().parts_uuid()->string_view(),
              log_header[1].message().parts_uuid()->string_view());

    EXPECT_EQ(log_header[0].message().parts_index(), 0);
    EXPECT_EQ(log_header[1].message().parts_index(), 1);
  }

  // Even though it doesn't make any difference here, exercise the logic for
  // passing in a separate config.
  LogReader reader(SortParts({logfile0, logfile1}), &config_.message());

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
  const ::std::string tmpdir = aos::testing::TestTmpDir();
  const ::std::string base_name = tmpdir + "/logfile";
  const ::std::string logfile = base_name + ".part0.bfbs";
  // Remove the log file.
  unlink(logfile.c_str());

  LOG(INFO) << "Logging data to " << logfile;
  ping_.set_quiet(true);

  {
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

    Logger logger(logger_event_loop.get());
    logger.set_polling_period(std::chrono::milliseconds(100));
    logger.StartLoggingLocalNamerOnRun(base_name);

    event_loop_factory_.RunFor(chrono::milliseconds(1000));
  }
}

std::vector<std::string> MakeLogFiles(std::string logfile_base) {
  return std::vector<std::string>(
      {logfile_base + "_pi1_data.part0.bfbs",
       logfile_base + "_pi2_data/test/aos.examples.Pong.part0.bfbs",
       logfile_base + "_pi2_data/test/aos.examples.Pong.part1.bfbs",
       logfile_base + "_pi2_data.part0.bfbs",
       logfile_base + "_timestamps/pi1/aos/remote_timestamps/pi2/"
                      "aos.message_bridge.RemoteMessage.part0.bfbs",
       logfile_base + "_timestamps/pi1/aos/remote_timestamps/pi2/"
                      "aos.message_bridge.RemoteMessage.part1.bfbs",
       logfile_base + "_timestamps/pi2/aos/remote_timestamps/pi1/"
                      "aos.message_bridge.RemoteMessage.part0.bfbs",
       logfile_base + "_timestamps/pi2/aos/remote_timestamps/pi1/"
                      "aos.message_bridge.RemoteMessage.part1.bfbs",
       logfile_base +
           "_pi1_data/pi1/aos/aos.message_bridge.Timestamp.part0.bfbs",
       logfile_base +
           "_pi1_data/pi1/aos/aos.message_bridge.Timestamp.part1.bfbs",
       logfile_base +
           "_pi2_data/pi2/aos/aos.message_bridge.Timestamp.part0.bfbs",
       logfile_base +
           "_pi2_data/pi2/aos/aos.message_bridge.Timestamp.part1.bfbs"});
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
        tmp_dir_(aos::testing::TestTmpDir()),
        logfile_base_(tmp_dir_ + "/multi_logfile"),
        pi1_reboot_logfiles_(
            {logfile_base_ + "_pi1_data.part0.bfbs",
             logfile_base_ + "_pi2_data/test/aos.examples.Pong.part0.bfbs",
             logfile_base_ + "_pi2_data/test/aos.examples.Pong.part1.bfbs",
             logfile_base_ + "_pi2_data/test/aos.examples.Pong.part2.bfbs",
             logfile_base_ + "_timestamps/pi1/aos/remote_timestamps/pi2/"
                             "aos.message_bridge.RemoteMessage.part0.bfbs",
             logfile_base_ + "_timestamps/pi1/aos/remote_timestamps/pi2/"
                             "aos.message_bridge.RemoteMessage.part1.bfbs",
             logfile_base_ + "_timestamps/pi1/aos/remote_timestamps/pi2/"
                             "aos.message_bridge.RemoteMessage.part2.bfbs",
             logfile_base_ +
                 "_pi2_data/pi2/aos/aos.message_bridge.Timestamp.part0.bfbs",
             logfile_base_ +
                 "_pi2_data/pi2/aos/aos.message_bridge.Timestamp.part1.bfbs",
             logfile_base_ +
                 "_pi2_data/pi2/aos/aos.message_bridge.Timestamp.part2.bfbs"}),
        logfiles_(MakeLogFiles(logfile_base_)),
        pi1_single_direction_logfiles_(
            {logfile_base_ + "_pi1_data.part0.bfbs",
             logfile_base_ + "_pi2_data/test/aos.examples.Pong.part0.bfbs",
             logfile_base_ + "_timestamps/pi1/aos/remote_timestamps/pi2/"
                             "aos.message_bridge.RemoteMessage.part0.bfbs",
             logfile_base_ +
                 "_pi2_data/pi2/aos/aos.message_bridge.Timestamp.part0.bfbs"}),
        structured_logfiles_{
            std::vector<std::string>{logfiles_[0]},
            std::vector<std::string>{logfiles_[1], logfiles_[2]},
            std::vector<std::string>{logfiles_[3]},
            std::vector<std::string>{logfiles_[4], logfiles_[5]},
            std::vector<std::string>{logfiles_[6], logfiles_[7]},
            std::vector<std::string>{logfiles_[8], logfiles_[9]},
            std::vector<std::string>{logfiles_[10], logfiles_[11]}},
        ping_event_loop_(event_loop_factory_.MakeEventLoop("ping", pi1_)),
        ping_(ping_event_loop_.get()),
        pong_event_loop_(event_loop_factory_.MakeEventLoop("pong", pi2_)),
        pong_(pong_event_loop_.get()) {
    // Go through and remove the logfiles if they already exist.
    for (const auto file : logfiles_) {
      unlink(file.c_str());
      unlink((file + ".xz").c_str());
    }

    for (const auto file : MakeLogFiles("relogged")) {
      unlink(file.c_str());
    }

    for (const auto file : pi1_reboot_logfiles_) {
      unlink(file.c_str());
    }

    LOG(INFO) << "Logging data to " << logfiles_[0] << ", " << logfiles_[1]
              << " and " << logfiles_[2];
  }

  struct LoggerState {
    std::unique_ptr<EventLoop> event_loop;
    std::unique_ptr<Logger> logger;
  };

  LoggerState MakeLogger(const Node *node,
                         SimulatedEventLoopFactory *factory = nullptr) {
    if (factory == nullptr) {
      factory = &event_loop_factory_;
    }
    return {factory->MakeEventLoop("logger", node), {}};
  }

  void StartLogger(LoggerState *logger, std::string logfile_base = "",
                   bool compress = false) {
    if (logfile_base.empty()) {
      logfile_base = logfile_base_;
    }

    logger->logger = std::make_unique<Logger>(logger->event_loop.get());
    logger->logger->set_polling_period(std::chrono::milliseconds(100));
    logger->event_loop->OnRun([logger, logfile_base, compress]() {
      std::unique_ptr<MultiNodeLogNamer> namer =
          std::make_unique<MultiNodeLogNamer>(
              logfile_base, logger->event_loop->configuration(),
              logger->event_loop->node());
      if (compress) {
#ifdef LZMA
        namer->set_extension(".xz");
        namer->set_encoder_factory(
            []() { return std::make_unique<aos::logger::LzmaEncoder>(3); });
#else
        LOG(FATAL) << "Compression unsupported";
#endif
      }

      logger->logger->StartLogging(std::move(namer));
    });
  }

  void VerifyParts(const std::vector<LogFile> &sorted_parts,
                   const std::vector<std::string> &corrupted_parts = {}) {
    EXPECT_EQ(sorted_parts.size(), 2u);

    // Count up the number of UUIDs and make sure they are what we expect as a
    // sanity check.
    std::set<std::string> log_event_uuids;
    std::set<std::string> parts_uuids;
    std::set<std::string> both_uuids;

    size_t missing_rt_count = 0;

    std::vector<std::string> logger_nodes;
    for (const LogFile &log_file : sorted_parts) {
      EXPECT_FALSE(log_file.log_event_uuid.empty());
      log_event_uuids.insert(log_file.log_event_uuid);
      logger_nodes.emplace_back(log_file.logger_node);
      both_uuids.insert(log_file.log_event_uuid);

      for (const LogParts &part : log_file.parts) {
        EXPECT_NE(part.monotonic_start_time, aos::monotonic_clock::min_time)
            << ": " << part;
        missing_rt_count +=
            part.realtime_start_time == aos::realtime_clock::min_time;

        EXPECT_TRUE(log_event_uuids.find(part.log_event_uuid) !=
                    log_event_uuids.end());
        EXPECT_NE(part.node, "");
        parts_uuids.insert(part.parts_uuid);
        both_uuids.insert(part.parts_uuid);
      }
    }

    // We won't have RT timestamps for 5 log files.  We don't log the RT start
    // time on remote nodes because we don't know it and would be guessing.  And
    // the log reader can actually do a better job.
    EXPECT_EQ(missing_rt_count, 5u);

    EXPECT_EQ(log_event_uuids.size(), 2u);
    EXPECT_EQ(parts_uuids.size(), ToLogReaderVector(sorted_parts).size());
    EXPECT_EQ(log_event_uuids.size() + parts_uuids.size(), both_uuids.size());

    // Test that each list of parts is in order.  Don't worry about the ordering
    // between part file lists though.
    // (inner vectors all need to be in order, but outer one doesn't matter).
    EXPECT_THAT(ToLogReaderVector(sorted_parts),
                ::testing::UnorderedElementsAreArray(structured_logfiles_));

    EXPECT_THAT(logger_nodes, ::testing::UnorderedElementsAre("pi1", "pi2"));

    EXPECT_NE(sorted_parts[0].realtime_start_time,
              aos::realtime_clock::min_time);
    EXPECT_NE(sorted_parts[1].realtime_start_time,
              aos::realtime_clock::min_time);

    EXPECT_NE(sorted_parts[0].monotonic_start_time,
              aos::monotonic_clock::min_time);
    EXPECT_NE(sorted_parts[1].monotonic_start_time,
              aos::monotonic_clock::min_time);

    EXPECT_THAT(sorted_parts[0].corrupted, ::testing::Eq(corrupted_parts));
    EXPECT_THAT(sorted_parts[1].corrupted, ::testing::Eq(corrupted_parts));
  }

  void ConfirmReadable(const std::vector<std::string> &files) {
    LogReader reader(SortParts(files));

    SimulatedEventLoopFactory log_reader_factory(reader.configuration());
    reader.Register(&log_reader_factory);

    log_reader_factory.Run();

    reader.Deregister();
  }

  void AddExtension(std::string_view extension) {
    std::transform(logfiles_.begin(), logfiles_.end(), logfiles_.begin(),
                   [extension](const std::string &in) {
                     return absl::StrCat(in, extension);
                   });

    std::transform(structured_logfiles_.begin(), structured_logfiles_.end(),
                   structured_logfiles_.begin(),
                   [extension](std::vector<std::string> in) {
                     std::transform(in.begin(), in.end(), in.begin(),
                                    [extension](const std::string &in_str) {
                                      return absl::StrCat(in_str, extension);
                                    });
                     return in;
                   });
  }

  // Config and factory.
  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  SimulatedEventLoopFactory event_loop_factory_;

  const Node *pi1_;
  const Node *pi2_;

  std::string tmp_dir_;
  std::string logfile_base_;
  std::vector<std::string> pi1_reboot_logfiles_;
  std::vector<std::string> logfiles_;
  std::vector<std::string> pi1_single_direction_logfiles_;

  std::vector<std::vector<std::string>> structured_logfiles_;

  std::unique_ptr<EventLoop> ping_event_loop_;
  Ping ping_;
  std::unique_ptr<EventLoop> pong_event_loop_;
  Pong pong_;
};

// Counts the number of messages on a channel.  Returns (channel name, channel
// type, count) for every message matching matcher()
std::vector<std::tuple<std::string, std::string, int>> CountChannelsMatching(
    std::string_view filename,
    std::function<bool(const MessageHeader *)> matcher) {
  MessageReader message_reader(filename);
  std::vector<int> counts(
      message_reader.log_file_header()->configuration()->channels()->size(), 0);

  while (true) {
    std::optional<SizePrefixedFlatbufferVector<MessageHeader>> msg =
        message_reader.ReadMessage();
    if (!msg) {
      break;
    }

    if (matcher(&msg.value().message())) {
      counts[msg.value().message().channel_index()]++;
    }
  }

  std::vector<std::tuple<std::string, std::string, int>> result;
  int channel = 0;
  for (size_t i = 0; i < counts.size(); ++i) {
    if (counts[i] != 0) {
      const Channel *channel =
          message_reader.log_file_header()->configuration()->channels()->Get(i);
      result.push_back(std::make_tuple(channel->name()->str(),
                                       channel->type()->str(), counts[i]));
    }
    ++channel;
  }

  return result;
}

// Counts the number of messages (channel, count) for all data messages.
std::vector<std::tuple<std::string, std::string, int>> CountChannelsData(
    std::string_view filename) {
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
std::vector<std::tuple<std::string, std::string, int>> CountChannelsTimestamp(
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
    std::set<std::string> logfile_uuids;
    std::set<std::string> parts_uuids;
    // Confirm that we have the expected number of UUIDs for both the logfile
    // UUIDs and parts UUIDs.
    std::vector<SizePrefixedFlatbufferVector<LogFileHeader>> log_header;
    for (std::string_view f : logfiles_) {
      log_header.emplace_back(ReadHeader(f).value());
      logfile_uuids.insert(log_header.back().message().log_event_uuid()->str());
      parts_uuids.insert(log_header.back().message().parts_uuid()->str());
    }

    EXPECT_EQ(logfile_uuids.size(), 2u);
    EXPECT_EQ(parts_uuids.size(), 7u);

    // And confirm everything is on the correct node.
    EXPECT_EQ(log_header[0].message().node()->name()->string_view(), "pi1");
    EXPECT_EQ(log_header[1].message().node()->name()->string_view(), "pi2");
    EXPECT_EQ(log_header[2].message().node()->name()->string_view(), "pi2");
    EXPECT_EQ(log_header[3].message().node()->name()->string_view(), "pi2");
    EXPECT_EQ(log_header[4].message().node()->name()->string_view(), "pi2");
    EXPECT_EQ(log_header[5].message().node()->name()->string_view(), "pi2");
    EXPECT_EQ(log_header[6].message().node()->name()->string_view(), "pi1");
    EXPECT_EQ(log_header[7].message().node()->name()->string_view(), "pi1");
    EXPECT_EQ(log_header[8].message().node()->name()->string_view(), "pi1");
    EXPECT_EQ(log_header[9].message().node()->name()->string_view(), "pi1");
    EXPECT_EQ(log_header[10].message().node()->name()->string_view(), "pi2");
    EXPECT_EQ(log_header[11].message().node()->name()->string_view(), "pi2");

    // And the parts index matches.
    EXPECT_EQ(log_header[0].message().parts_index(), 0);
    EXPECT_EQ(log_header[1].message().parts_index(), 0);
    EXPECT_EQ(log_header[2].message().parts_index(), 1);
    EXPECT_EQ(log_header[3].message().parts_index(), 0);
    EXPECT_EQ(log_header[4].message().parts_index(), 0);
    EXPECT_EQ(log_header[5].message().parts_index(), 1);
    EXPECT_EQ(log_header[6].message().parts_index(), 0);
    EXPECT_EQ(log_header[7].message().parts_index(), 1);
    EXPECT_EQ(log_header[8].message().parts_index(), 0);
    EXPECT_EQ(log_header[9].message().parts_index(), 1);
    EXPECT_EQ(log_header[10].message().parts_index(), 0);
    EXPECT_EQ(log_header[11].message().parts_index(), 1);
  }

  {
    using ::testing::UnorderedElementsAre;

    // Timing reports, pings
    EXPECT_THAT(
        CountChannelsData(logfiles_[0]),
        UnorderedElementsAre(
            std::make_tuple("/pi1/aos", "aos.message_bridge.Timestamp", 200),
            std::make_tuple("/pi1/aos", "aos.timing.Report", 40),
            std::make_tuple("/test", "aos.examples.Ping", 2001)))
        << " : " << logfiles_[0];
    // Timestamps for pong
    EXPECT_THAT(
        CountChannelsTimestamp(logfiles_[0]),
        UnorderedElementsAre(
            std::make_tuple("/test", "aos.examples.Pong", 2001),
            std::make_tuple("/pi2/aos", "aos.message_bridge.Timestamp", 200)))
        << " : " << logfiles_[0];

    // Pong data.
    EXPECT_THAT(
        CountChannelsData(logfiles_[1]),
        UnorderedElementsAre(std::make_tuple("/test", "aos.examples.Pong", 91)))
        << " : " << logfiles_[1];
    EXPECT_THAT(CountChannelsData(logfiles_[2]),
                UnorderedElementsAre(
                    std::make_tuple("/test", "aos.examples.Pong", 1910)))
        << " : " << logfiles_[1];

    // No timestamps
    EXPECT_THAT(CountChannelsTimestamp(logfiles_[1]), UnorderedElementsAre())
        << " : " << logfiles_[1];
    EXPECT_THAT(CountChannelsTimestamp(logfiles_[2]), UnorderedElementsAre())
        << " : " << logfiles_[2];

    // Timing reports and pongs.
    EXPECT_THAT(
        CountChannelsData(logfiles_[3]),
        UnorderedElementsAre(
            std::make_tuple("/pi2/aos", "aos.message_bridge.Timestamp", 200),
            std::make_tuple("/pi2/aos", "aos.timing.Report", 40),
            std::make_tuple("/test", "aos.examples.Pong", 2001)))
        << " : " << logfiles_[3];
    // And ping timestamps.
    EXPECT_THAT(
        CountChannelsTimestamp(logfiles_[3]),
        UnorderedElementsAre(
            std::make_tuple("/test", "aos.examples.Ping", 2001),
            std::make_tuple("/pi1/aos", "aos.message_bridge.Timestamp", 200)))
        << " : " << logfiles_[3];

    // Timestamps from pi2 on pi1, and the other way.
    EXPECT_THAT(CountChannelsData(logfiles_[4]), UnorderedElementsAre())
        << " : " << logfiles_[4];
    EXPECT_THAT(CountChannelsData(logfiles_[5]), UnorderedElementsAre())
        << " : " << logfiles_[5];
    EXPECT_THAT(CountChannelsData(logfiles_[6]), UnorderedElementsAre())
        << " : " << logfiles_[6];
    EXPECT_THAT(CountChannelsData(logfiles_[7]), UnorderedElementsAre())
        << " : " << logfiles_[7];
    EXPECT_THAT(
        CountChannelsTimestamp(logfiles_[4]),
        UnorderedElementsAre(
            std::make_tuple("/pi1/aos", "aos.message_bridge.Timestamp", 9),
            std::make_tuple("/test", "aos.examples.Ping", 91)))
        << " : " << logfiles_[4];
    EXPECT_THAT(
        CountChannelsTimestamp(logfiles_[5]),
        UnorderedElementsAre(
            std::make_tuple("/pi1/aos", "aos.message_bridge.Timestamp", 191),
            std::make_tuple("/test", "aos.examples.Ping", 1910)))
        << " : " << logfiles_[5];
    EXPECT_THAT(CountChannelsTimestamp(logfiles_[6]),
                UnorderedElementsAre(std::make_tuple(
                    "/pi2/aos", "aos.message_bridge.Timestamp", 9)))
        << " : " << logfiles_[6];
    EXPECT_THAT(CountChannelsTimestamp(logfiles_[7]),
                UnorderedElementsAre(std::make_tuple(
                    "/pi2/aos", "aos.message_bridge.Timestamp", 191)))
        << " : " << logfiles_[7];

    // And then test that the remotely logged timestamp data files only have
    // timestamps in them.
    EXPECT_THAT(CountChannelsTimestamp(logfiles_[8]), UnorderedElementsAre())
        << " : " << logfiles_[8];
    EXPECT_THAT(CountChannelsTimestamp(logfiles_[9]), UnorderedElementsAre())
        << " : " << logfiles_[9];
    EXPECT_THAT(CountChannelsTimestamp(logfiles_[10]), UnorderedElementsAre())
        << " : " << logfiles_[10];
    EXPECT_THAT(CountChannelsTimestamp(logfiles_[11]), UnorderedElementsAre())
        << " : " << logfiles_[11];

    EXPECT_THAT(CountChannelsData(logfiles_[8]),
                UnorderedElementsAre(std::make_tuple(
                    "/pi1/aos", "aos.message_bridge.Timestamp", 9)))
        << " : " << logfiles_[8];
    EXPECT_THAT(CountChannelsData(logfiles_[9]),
                UnorderedElementsAre(std::make_tuple(
                    "/pi1/aos", "aos.message_bridge.Timestamp", 191)))
        << " : " << logfiles_[9];

    EXPECT_THAT(CountChannelsData(logfiles_[10]),
                UnorderedElementsAre(std::make_tuple(
                    "/pi2/aos", "aos.message_bridge.Timestamp", 9)))
        << " : " << logfiles_[10];
    EXPECT_THAT(CountChannelsData(logfiles_[11]),
                UnorderedElementsAre(std::make_tuple(
                    "/pi2/aos", "aos.message_bridge.Timestamp", 191)))
        << " : " << logfiles_[11];
  }

  LogReader reader(SortParts(logfiles_));

  SimulatedEventLoopFactory log_reader_factory(reader.configuration());
  log_reader_factory.set_send_delay(chrono::microseconds(0));

  // This sends out the fetched messages and advances time to the start of the
  // log file.
  reader.Register(&log_reader_factory);

  const Node *pi1 =
      configuration::GetNode(log_reader_factory.configuration(), "pi1");
  const Node *pi2 =
      configuration::GetNode(log_reader_factory.configuration(), "pi2");

  LOG(INFO) << "Start time " << reader.monotonic_start_time(pi1) << " pi1";
  LOG(INFO) << "Start time " << reader.monotonic_start_time(pi2) << " pi2";
  LOG(INFO) << "now pi1 "
            << log_reader_factory.GetNodeEventLoopFactory(pi1)->monotonic_now();
  LOG(INFO) << "now pi2 "
            << log_reader_factory.GetNodeEventLoopFactory(pi2)->monotonic_now();

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
        VLOG(1) << "Pi1 ping " << FlatbufferToJson(&ping) << " at "
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
        VLOG(1) << "Pi2 ping " << FlatbufferToJson(&ping) << " at "
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

  constexpr ssize_t kQueueIndexOffset = -9;
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
                  pi2_pong_count + kQueueIndexOffset);

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

  const std::vector<LogFile> sorted_parts = SortParts(logfiles_);
  EXPECT_DEATH(LogReader(sorted_parts, &extra_nodes_config.message()),
               "Log file and replay config need to have matching nodes lists.");
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

  LogReader reader(SortParts(logfiles_));

  SimulatedEventLoopFactory log_reader_factory(reader.configuration());
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

    pi2->SetDistributedOffset(-pi2_offset, 1.0);
    LOG(INFO) << "pi2 times: " << pi2->monotonic_now() << " "
              << pi2->realtime_now() << " distributed "
              << pi2->ToDistributedClock(pi2->monotonic_now());

    for (int i = 0; i < 95; ++i) {
      pi2_offset += chrono::nanoseconds(200);
      pi2->SetDistributedOffset(-pi2_offset, 1.0);
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
        pi2->SetDistributedOffset(-pi2_offset, 1.0);
        event_loop_factory_.RunFor(chrono::milliseconds(1));
      }

      EXPECT_GT(pi2_offset - initial_pi2_offset,
                event_loop_factory_.send_delay() +
                    event_loop_factory_.network_delay());

      for (int i = 0; i < 40000; ++i) {
        pi2_offset -= chrono::nanoseconds(200);
        pi2->SetDistributedOffset(-pi2_offset, 1.0);
        event_loop_factory_.RunFor(chrono::milliseconds(1));
      }
    }

    // And log a bit more on pi2.
    event_loop_factory_.RunFor(chrono::milliseconds(400));
  }

  LogReader reader(SortParts(logfiles_));

  SimulatedEventLoopFactory log_reader_factory(reader.configuration());
  log_reader_factory.set_send_delay(chrono::microseconds(0));

  const Node *pi1 =
      configuration::GetNode(log_reader_factory.configuration(), "pi1");
  const Node *pi2 =
      configuration::GetNode(log_reader_factory.configuration(), "pi2");

  // This sends out the fetched messages and advances time to the start of the
  // log file.
  reader.Register(&log_reader_factory);

  LOG(INFO) << "Start time " << reader.monotonic_start_time(pi1) << " pi1";
  LOG(INFO) << "Start time " << reader.monotonic_start_time(pi2) << " pi2";
  LOG(INFO) << "now pi1 "
            << log_reader_factory.GetNodeEventLoopFactory(pi1)->monotonic_now();
  LOG(INFO) << "now pi2 "
            << log_reader_factory.GetNodeEventLoopFactory(pi2)->monotonic_now();

  LOG(INFO) << "Done registering (pi1) "
            << log_reader_factory.GetNodeEventLoopFactory(pi1)->monotonic_now()
            << " "
            << log_reader_factory.GetNodeEventLoopFactory(pi1)->realtime_now();
  LOG(INFO) << "Done registering (pi2) "
            << log_reader_factory.GetNodeEventLoopFactory(pi2)->monotonic_now()
            << " "
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

// Tests that we can sort a bunch of parts into the pre-determined sorted parts.
TEST_F(MultinodeLoggerTest, SortParts) {
  // Make a bunch of parts.
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(2000));
  }

  const std::vector<LogFile> sorted_parts = SortParts(logfiles_);
  VerifyParts(sorted_parts);
}

// Tests that we can sort a bunch of parts with an empty part.  We should ignore
// it and remove it from the sorted list.
TEST_F(MultinodeLoggerTest, SortEmptyParts) {
  // Make a bunch of parts.
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(2000));
  }

  // TODO(austin): Should we flip out if the file can't open?
  const std::string kEmptyFile("foobarinvalidfiledoesnotexist.bfbs");

  aos::util::WriteStringToFileOrDie(kEmptyFile, "");
  logfiles_.emplace_back(kEmptyFile);

  const std::vector<LogFile> sorted_parts = SortParts(logfiles_);
  VerifyParts(sorted_parts, {kEmptyFile});
}

#ifdef LZMA
// Tests that we can sort a bunch of parts with an empty .xz file in there.  The
// empty file should be ignored.
TEST_F(MultinodeLoggerTest, SortEmptyCompressedParts) {
  // Make a bunch of parts.
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger, "", true);
    StartLogger(&pi2_logger, "", true);

    event_loop_factory_.RunFor(chrono::milliseconds(2000));
  }

  // TODO(austin): Should we flip out if the file can't open?
  const std::string kEmptyFile("foobarinvalidfiledoesnotexist.bfbs.xz");

  AddExtension(".xz");

  aos::util::WriteStringToFileOrDie(kEmptyFile, "");
  logfiles_.emplace_back(kEmptyFile);

  const std::vector<LogFile> sorted_parts = SortParts(logfiles_);
  VerifyParts(sorted_parts, {kEmptyFile});
}

// Tests that we can sort a bunch of parts with the end missing off a compressed
// file.  We should use the part we can read.
TEST_F(MultinodeLoggerTest, SortTruncatedCompressedParts) {
  // Make a bunch of parts.
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger, "", true);
    StartLogger(&pi2_logger, "", true);

    event_loop_factory_.RunFor(chrono::milliseconds(2000));
  }

  // Append everything with .xz.
  AddExtension(".xz");

  // Strip off the end of one of the files.  Pick one with a lot of data.
  ::std::string compressed_contents =
      aos::util::ReadFileToStringOrDie(logfiles_[0]);

  aos::util::WriteStringToFileOrDie(
      logfiles_[0],
      compressed_contents.substr(0, compressed_contents.size() - 100));

  const std::vector<LogFile> sorted_parts = SortParts(logfiles_);
  VerifyParts(sorted_parts);
}
#endif

// Tests that if we remap a remapped channel, it shows up correctly.
TEST_F(MultinodeLoggerTest, RemapLoggedChannel) {
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }

  LogReader reader(SortParts(logfiles_));

  // Remap just on pi1.
  reader.RemapLoggedChannel<aos::timing::Report>(
      "/aos", configuration::GetNode(reader.configuration(), "pi1"));

  SimulatedEventLoopFactory log_reader_factory(reader.configuration());
  log_reader_factory.set_send_delay(chrono::microseconds(0));

  reader.Register(&log_reader_factory);

  const Node *pi1 =
      configuration::GetNode(log_reader_factory.configuration(), "pi1");
  const Node *pi2 =
      configuration::GetNode(log_reader_factory.configuration(), "pi2");

  // Confirm we can read the data on the remapped channel, just for pi1. Nothing
  // else should have moved.
  std::unique_ptr<EventLoop> pi1_event_loop =
      log_reader_factory.MakeEventLoop("test", pi1);
  pi1_event_loop->SkipTimingReport();
  std::unique_ptr<EventLoop> full_pi1_event_loop =
      log_reader_factory.MakeEventLoop("test", pi1);
  full_pi1_event_loop->SkipTimingReport();
  std::unique_ptr<EventLoop> pi2_event_loop =
      log_reader_factory.MakeEventLoop("test", pi2);
  pi2_event_loop->SkipTimingReport();

  MessageCounter<aos::timing::Report> pi1_timing_report(pi1_event_loop.get(),
                                                        "/aos");
  MessageCounter<aos::timing::Report> full_pi1_timing_report(
      full_pi1_event_loop.get(), "/pi1/aos");
  MessageCounter<aos::timing::Report> pi1_original_timing_report(
      pi1_event_loop.get(), "/original/aos");
  MessageCounter<aos::timing::Report> full_pi1_original_timing_report(
      full_pi1_event_loop.get(), "/original/pi1/aos");
  MessageCounter<aos::timing::Report> pi2_timing_report(pi2_event_loop.get(),
                                                        "/aos");

  log_reader_factory.Run();

  EXPECT_EQ(pi1_timing_report.count(), 0u);
  EXPECT_EQ(full_pi1_timing_report.count(), 0u);
  EXPECT_NE(pi1_original_timing_report.count(), 0u);
  EXPECT_NE(full_pi1_original_timing_report.count(), 0u);
  EXPECT_NE(pi2_timing_report.count(), 0u);

  reader.Deregister();
}

// Tests that we properly recreate forwarded timestamps when replaying a log.
// This should be enough that we can then re-run the logger and get a valid log
// back.
TEST_F(MultinodeLoggerTest, MessageHeader) {
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }

  LogReader reader(SortParts(logfiles_));

  SimulatedEventLoopFactory log_reader_factory(reader.configuration());
  log_reader_factory.set_send_delay(chrono::microseconds(0));

  // This sends out the fetched messages and advances time to the start of the
  // log file.
  reader.Register(&log_reader_factory);

  const Node *pi1 =
      configuration::GetNode(log_reader_factory.configuration(), "pi1");
  const Node *pi2 =
      configuration::GetNode(log_reader_factory.configuration(), "pi2");

  LOG(INFO) << "Start time " << reader.monotonic_start_time(pi1) << " pi1";
  LOG(INFO) << "Start time " << reader.monotonic_start_time(pi2) << " pi2";
  LOG(INFO) << "now pi1 "
            << log_reader_factory.GetNodeEventLoopFactory(pi1)->monotonic_now();
  LOG(INFO) << "now pi2 "
            << log_reader_factory.GetNodeEventLoopFactory(pi2)->monotonic_now();

  EXPECT_THAT(reader.Nodes(), ::testing::ElementsAre(pi1, pi2));

  reader.event_loop_factory()->set_send_delay(chrono::microseconds(0));

  std::unique_ptr<EventLoop> pi1_event_loop =
      log_reader_factory.MakeEventLoop("test", pi1);
  std::unique_ptr<EventLoop> pi2_event_loop =
      log_reader_factory.MakeEventLoop("test", pi2);

  MessageCounter<RemoteMessage> pi1_original_message_header_counter(
      pi1_event_loop.get(), "/original/aos/remote_timestamps/pi2");
  MessageCounter<RemoteMessage> pi2_original_message_header_counter(
      pi2_event_loop.get(), "/original/aos/remote_timestamps/pi1");

  aos::Fetcher<message_bridge::Timestamp> pi1_timestamp_on_pi1_fetcher =
      pi1_event_loop->MakeFetcher<message_bridge::Timestamp>("/pi1/aos");
  aos::Fetcher<message_bridge::Timestamp> pi1_timestamp_on_pi2_fetcher =
      pi2_event_loop->MakeFetcher<message_bridge::Timestamp>("/pi1/aos");

  aos::Fetcher<examples::Ping> ping_on_pi1_fetcher =
      pi1_event_loop->MakeFetcher<examples::Ping>("/test");
  aos::Fetcher<examples::Ping> ping_on_pi2_fetcher =
      pi2_event_loop->MakeFetcher<examples::Ping>("/test");

  aos::Fetcher<message_bridge::Timestamp> pi2_timestamp_on_pi2_fetcher =
      pi2_event_loop->MakeFetcher<message_bridge::Timestamp>("/pi2/aos");
  aos::Fetcher<message_bridge::Timestamp> pi2_timestamp_on_pi1_fetcher =
      pi1_event_loop->MakeFetcher<message_bridge::Timestamp>("/pi2/aos");

  aos::Fetcher<examples::Pong> pong_on_pi2_fetcher =
      pi2_event_loop->MakeFetcher<examples::Pong>("/test");
  aos::Fetcher<examples::Pong> pong_on_pi1_fetcher =
      pi1_event_loop->MakeFetcher<examples::Pong>("/test");

  const size_t pi1_timestamp_channel = configuration::ChannelIndex(
      pi1_event_loop->configuration(), pi1_timestamp_on_pi1_fetcher.channel());
  const size_t ping_timestamp_channel = configuration::ChannelIndex(
      pi2_event_loop->configuration(), ping_on_pi2_fetcher.channel());

  const size_t pi2_timestamp_channel = configuration::ChannelIndex(
      pi2_event_loop->configuration(), pi2_timestamp_on_pi2_fetcher.channel());
  const size_t pong_timestamp_channel = configuration::ChannelIndex(
      pi1_event_loop->configuration(), pong_on_pi1_fetcher.channel());

  pi1_event_loop->MakeWatcher(
      "/aos/remote_timestamps/pi2",
      [&pi1_event_loop, &pi2_event_loop, pi1_timestamp_channel,
       ping_timestamp_channel, &pi1_timestamp_on_pi1_fetcher,
       &pi1_timestamp_on_pi2_fetcher, &ping_on_pi1_fetcher,
       &ping_on_pi2_fetcher](const RemoteMessage &header) {
        const aos::monotonic_clock::time_point header_monotonic_sent_time(
            chrono::nanoseconds(header.monotonic_sent_time()));
        const aos::realtime_clock::time_point header_realtime_sent_time(
            chrono::nanoseconds(header.realtime_sent_time()));
        const aos::monotonic_clock::time_point header_monotonic_remote_time(
            chrono::nanoseconds(header.monotonic_remote_time()));
        const aos::realtime_clock::time_point header_realtime_remote_time(
            chrono::nanoseconds(header.realtime_remote_time()));

        const Context *pi1_context = nullptr;
        const Context *pi2_context = nullptr;

        if (header.channel_index() == pi1_timestamp_channel) {
          ASSERT_TRUE(pi1_timestamp_on_pi1_fetcher.FetchNext());
          ASSERT_TRUE(pi1_timestamp_on_pi2_fetcher.FetchNext());
          pi1_context = &pi1_timestamp_on_pi1_fetcher.context();
          pi2_context = &pi1_timestamp_on_pi2_fetcher.context();
        } else if (header.channel_index() == ping_timestamp_channel) {
          ASSERT_TRUE(ping_on_pi1_fetcher.FetchNext());
          ASSERT_TRUE(ping_on_pi2_fetcher.FetchNext());
          pi1_context = &ping_on_pi1_fetcher.context();
          pi2_context = &ping_on_pi2_fetcher.context();
        } else {
          LOG(FATAL) << "Unknown channel " << FlatbufferToJson(&header) << " "
                     << configuration::CleanedChannelToString(
                            pi1_event_loop->configuration()->channels()->Get(
                                header.channel_index()));
        }

        ASSERT_TRUE(header.has_boot_uuid());
        EXPECT_EQ(header.boot_uuid()->string_view(),
                  pi2_event_loop->boot_uuid().string_view());

        EXPECT_EQ(pi1_context->queue_index, header.remote_queue_index());
        EXPECT_EQ(pi2_context->remote_queue_index, header.remote_queue_index());
        EXPECT_EQ(pi2_context->queue_index, header.queue_index());

        EXPECT_EQ(pi2_context->monotonic_event_time,
                  header_monotonic_sent_time);
        EXPECT_EQ(pi2_context->realtime_event_time, header_realtime_sent_time);
        EXPECT_EQ(pi2_context->realtime_remote_time,
                  header_realtime_remote_time);
        EXPECT_EQ(pi2_context->monotonic_remote_time,
                  header_monotonic_remote_time);

        EXPECT_EQ(pi1_context->realtime_event_time,
                  header_realtime_remote_time);
        EXPECT_EQ(pi1_context->monotonic_event_time,
                  header_monotonic_remote_time);
      });
  pi2_event_loop->MakeWatcher(
      "/aos/remote_timestamps/pi1",
      [&pi2_event_loop, &pi1_event_loop, pi2_timestamp_channel,
       pong_timestamp_channel, &pi2_timestamp_on_pi2_fetcher,
       &pi2_timestamp_on_pi1_fetcher, &pong_on_pi2_fetcher,
       &pong_on_pi1_fetcher](const RemoteMessage &header) {
        const aos::monotonic_clock::time_point header_monotonic_sent_time(
            chrono::nanoseconds(header.monotonic_sent_time()));
        const aos::realtime_clock::time_point header_realtime_sent_time(
            chrono::nanoseconds(header.realtime_sent_time()));
        const aos::monotonic_clock::time_point header_monotonic_remote_time(
            chrono::nanoseconds(header.monotonic_remote_time()));
        const aos::realtime_clock::time_point header_realtime_remote_time(
            chrono::nanoseconds(header.realtime_remote_time()));

        const Context *pi2_context = nullptr;
        const Context *pi1_context = nullptr;

        if (header.channel_index() == pi2_timestamp_channel) {
          ASSERT_TRUE(pi2_timestamp_on_pi2_fetcher.FetchNext());
          ASSERT_TRUE(pi2_timestamp_on_pi1_fetcher.FetchNext());
          pi2_context = &pi2_timestamp_on_pi2_fetcher.context();
          pi1_context = &pi2_timestamp_on_pi1_fetcher.context();
        } else if (header.channel_index() == pong_timestamp_channel) {
          ASSERT_TRUE(pong_on_pi2_fetcher.FetchNext());
          ASSERT_TRUE(pong_on_pi1_fetcher.FetchNext());
          pi2_context = &pong_on_pi2_fetcher.context();
          pi1_context = &pong_on_pi1_fetcher.context();
        } else {
          LOG(FATAL) << "Unknown channel " << FlatbufferToJson(&header) << " "
                     << configuration::CleanedChannelToString(
                            pi2_event_loop->configuration()->channels()->Get(
                                header.channel_index()));
        }

        ASSERT_TRUE(header.has_boot_uuid());
        EXPECT_EQ(header.boot_uuid()->string_view(),
                  pi1_event_loop->boot_uuid().string_view());

        EXPECT_EQ(pi2_context->queue_index, header.remote_queue_index());
        EXPECT_EQ(pi1_context->remote_queue_index, header.remote_queue_index());
        EXPECT_EQ(pi1_context->queue_index, header.queue_index());

        EXPECT_EQ(pi1_context->monotonic_event_time,
                  header_monotonic_sent_time);
        EXPECT_EQ(pi1_context->realtime_event_time, header_realtime_sent_time);
        EXPECT_EQ(pi1_context->realtime_remote_time,
                  header_realtime_remote_time);
        EXPECT_EQ(pi1_context->monotonic_remote_time,
                  header_monotonic_remote_time);

        EXPECT_EQ(pi2_context->realtime_event_time,
                  header_realtime_remote_time);
        EXPECT_EQ(pi2_context->monotonic_event_time,
                  header_monotonic_remote_time);
      });

  // And confirm we can re-create a log again, while checking the contents.
  {
    LoggerState pi1_logger = MakeLogger(
        configuration::GetNode(log_reader_factory.configuration(), pi1_),
        &log_reader_factory);
    LoggerState pi2_logger = MakeLogger(
        configuration::GetNode(log_reader_factory.configuration(), pi2_),
        &log_reader_factory);

    StartLogger(&pi1_logger, "relogged");
    StartLogger(&pi2_logger, "relogged");

    log_reader_factory.Run();
  }

  EXPECT_EQ(pi2_original_message_header_counter.count(), 0u);
  EXPECT_EQ(pi1_original_message_header_counter.count(), 0u);

  reader.Deregister();
}

// Tests that we properly populate and extract the logger_start time by setting
// up a clock difference between 2 nodes and looking at the resulting parts.
TEST_F(MultinodeLoggerTest, LoggerStartTime) {
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    NodeEventLoopFactory *pi2 =
        event_loop_factory_.GetNodeEventLoopFactory(pi2_);

    pi2->SetDistributedOffset(chrono::seconds(1000), 1.0);

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(10000));
  }

  for (const LogFile &log_file : SortParts(logfiles_)) {
    for (const LogParts &log_part : log_file.parts) {
      if (log_part.node == log_file.logger_node) {
        EXPECT_EQ(log_part.logger_monotonic_start_time,
                  aos::monotonic_clock::min_time);
        EXPECT_EQ(log_part.logger_realtime_start_time,
                  aos::realtime_clock::min_time);
      } else {
        const chrono::seconds offset = log_file.logger_node == "pi1"
                                           ? -chrono::seconds(1000)
                                           : chrono::seconds(1000);
        EXPECT_EQ(log_part.logger_monotonic_start_time,
                  log_part.monotonic_start_time + offset);
        EXPECT_EQ(log_part.logger_realtime_start_time,
                  log_file.realtime_start_time +
                      (log_part.logger_monotonic_start_time -
                       log_file.monotonic_start_time));
      }
    }
  }
}

// TODO(austin): We can write a test which recreates a logfile and confirms that
// we get it back.  That is the ultimate test.

// Tests that we properly recreate forwarded timestamps when replaying a log.
// This should be enough that we can then re-run the logger and get a valid log
// back.
TEST_F(MultinodeLoggerDeathTest, RemoteReboot) {
  std::string pi2_boot1;
  std::string pi2_boot2;
  {
    pi2_boot1 = event_loop_factory_.GetNodeEventLoopFactory(pi2_)
                    ->boot_uuid()
                    .string_view();
    LoggerState pi1_logger = MakeLogger(pi1_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(10000));

    event_loop_factory_.GetNodeEventLoopFactory(pi2_)->Reboot();

    pi2_boot2 = event_loop_factory_.GetNodeEventLoopFactory(pi2_)
                    ->boot_uuid()
                    .string_view();

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }

  // Confirm that we refuse to replay logs with missing boot uuids.
  EXPECT_DEATH(
      {
        LogReader reader(SortParts(pi1_reboot_logfiles_));

        SimulatedEventLoopFactory log_reader_factory(reader.configuration());
        log_reader_factory.set_send_delay(chrono::microseconds(0));

        // This sends out the fetched messages and advances time to the start of
        // the log file.
        reader.Register(&log_reader_factory);
      },
      absl::StrFormat("(%s|%s).*(%s|%s).*Found parts from different boots",
                      pi2_boot1, pi2_boot2, pi2_boot2, pi2_boot1));
}

// Tests that we properly handle one direction of message_bridge being
// unavailable.
TEST_F(MultinodeLoggerTest, OneDirectionWithNegativeSlope) {
  event_loop_factory_.GetNodeEventLoopFactory(pi1_)->Disconnect(pi2_);
  event_loop_factory_.GetNodeEventLoopFactory(pi2_)->SetDistributedOffset(
      chrono::seconds(1000), 0.99999);
  {
    LoggerState pi1_logger = MakeLogger(pi1_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(10000));
  }

  // Confirm that we can parse the result.  LogReader has enough internal CHECKs
  // to confirm the right thing happened.
  ConfirmReadable(pi1_single_direction_logfiles_);
}

// Tests that we properly handle one direction of message_bridge being
// unavailable.
TEST_F(MultinodeLoggerTest, OneDirectionWithPositiveSlope) {
  event_loop_factory_.GetNodeEventLoopFactory(pi1_)->Disconnect(pi2_);
  event_loop_factory_.GetNodeEventLoopFactory(pi2_)->SetDistributedOffset(
      chrono::seconds(500), 1.00001);
  {
    LoggerState pi1_logger = MakeLogger(pi1_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(10000));
  }

  // Confirm that we can parse the result.  LogReader has enough internal CHECKs
  // to confirm the right thing happened.
  ConfirmReadable(pi1_single_direction_logfiles_);
}

}  // namespace testing
}  // namespace logger
}  // namespace aos
