#include "aos/events/logging/log_reader.h"

#include <sys/stat.h>

#include "absl/strings/str_format.h"
#include "aos/events/event_loop.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/message_counter.h"
#include "aos/events/ping_lib.h"
#include "aos/events/pong_lib.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/remote_message_generated.h"
#include "aos/network/testing_time_converter.h"
#include "aos/network/timestamp_generated.h"
#include "aos/testing/path.h"
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

using aos::testing::ArtifactPath;

namespace chrono = std::chrono;
using aos::message_bridge::RemoteMessage;
using aos::testing::MessageCounter;

constexpr std::string_view kSingleConfigSha256(
    "bbe1b563139273b23a5405eebc2f2740cefcda5f96681acd0a84b8ff9ab93ea4");

std::vector<std::vector<std::string>> ToLogReaderVector(
    const std::vector<LogFile> &log_files) {
  std::vector<std::vector<std::string>> result;
  for (const LogFile &log_file : log_files) {
    for (const LogParts &log_parts : log_file.parts) {
      std::vector<std::string> parts;
      for (const std::string &part : log_parts.parts) {
        parts.emplace_back(part);
      }
      result.emplace_back(std::move(parts));
    }
  }
  return result;
}

class LoggerTest : public ::testing::Test {
 public:
  LoggerTest()
      : config_(aos::configuration::ReadConfig(
            ArtifactPath("aos/events/pingpong_config.json"))),
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
  const ::std::string config =
      absl::StrCat(base_name, kSingleConfigSha256, ".bfbs");
  const ::std::string logfile = base_name + ".part0.bfbs";
  // Remove it.
  unlink(config.c_str());
  unlink(logfile.c_str());

  LOG(INFO) << "Logging data to " << logfile;

  {
    std::unique_ptr<EventLoop> logger_event_loop =
        event_loop_factory_.MakeEventLoop("logger");

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    Logger logger(logger_event_loop.get());
    logger.set_separate_config(false);
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

  EXPECT_THAT(reader.LoggedNodes(), ::testing::ElementsAre(nullptr));

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
  const ::std::string config1 =
      absl::StrCat(base_name1, kSingleConfigSha256, ".bfbs");
  const ::std::string logfile1 = base_name1 + ".part0.bfbs";
  const ::std::string base_name2 = tmpdir + "/logfile2";
  const ::std::string config2 =
      absl::StrCat(base_name2, kSingleConfigSha256, ".bfbs");
  const ::std::string logfile2 = base_name2 + ".part0.bfbs";
  unlink(logfile1.c_str());
  unlink(config1.c_str());
  unlink(logfile2.c_str());
  unlink(config2.c_str());

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
              base_name1, logger_event_loop.get(), logger_event_loop->node()));
          EXPECT_DEATH(logger.StartLogging(std::make_unique<LocalLogNamer>(
                           base_name2, logger_event_loop.get(),
                           logger_event_loop->node())),
                       "Already logging");
        });
    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }
}

// Tests calling StopLogging twice.
TEST_F(LoggerDeathTest, ExtraStop) {
  const ::std::string tmpdir = aos::testing::TestTmpDir();
  const ::std::string base_name = tmpdir + "/logfile";
  const ::std::string config =
      absl::StrCat(base_name, kSingleConfigSha256, ".bfbs");
  const ::std::string logfile = base_name + ".part0.bfbs";
  // Remove it.
  unlink(config.c_str());
  unlink(logfile.c_str());

  LOG(INFO) << "Logging data to " << logfile;

  {
    std::unique_ptr<EventLoop> logger_event_loop =
        event_loop_factory_.MakeEventLoop("logger");

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    Logger logger(logger_event_loop.get());
    logger.set_separate_config(false);
    logger.set_polling_period(std::chrono::milliseconds(100));
    logger_event_loop->OnRun([base_name, &logger_event_loop, &logger]() {
      logger.StartLogging(std::make_unique<LocalLogNamer>(
          base_name, logger_event_loop.get(), logger_event_loop->node()));
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
  const ::std::string config1 =
      absl::StrCat(base_name1, kSingleConfigSha256, ".bfbs");
  const ::std::string logfile1 = base_name1 + ".part0.bfbs";
  const ::std::string base_name2 = tmpdir + "/logfile2";
  const ::std::string config2 =
      absl::StrCat(base_name2, kSingleConfigSha256, ".bfbs");
  const ::std::string logfile2 = base_name2 + ".part0.bfbs";
  unlink(logfile1.c_str());
  unlink(config1.c_str());
  unlink(logfile2.c_str());
  unlink(config2.c_str());

  LOG(INFO) << "Logging data to " << logfile1 << " then " << logfile2;

  {
    std::unique_ptr<EventLoop> logger_event_loop =
        event_loop_factory_.MakeEventLoop("logger");

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    Logger logger(logger_event_loop.get());
    logger.set_separate_config(false);
    logger.set_polling_period(std::chrono::milliseconds(100));
    logger.StartLogging(std::make_unique<LocalLogNamer>(
        base_name1, logger_event_loop.get(), logger_event_loop->node()));
    event_loop_factory_.RunFor(chrono::milliseconds(10000));
    logger.StopLogging(logger_event_loop->monotonic_now());
    event_loop_factory_.RunFor(chrono::milliseconds(10000));
    logger.StartLogging(std::make_unique<LocalLogNamer>(
        base_name2, logger_event_loop.get(), logger_event_loop->node()));
    event_loop_factory_.RunFor(chrono::milliseconds(10000));
  }

  for (const auto &logfile :
       {std::make_tuple(logfile1, 10), std::make_tuple(logfile2, 2010)}) {
    SCOPED_TRACE(std::get<0>(logfile));
    LogReader reader(std::get<0>(logfile));
    reader.Register();

    EXPECT_THAT(reader.LoggedNodes(), ::testing::ElementsAre(nullptr));

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
  const ::std::string config =
      absl::StrCat(base_name, kSingleConfigSha256, ".bfbs");
  const ::std::string logfile0 = base_name + ".part0.bfbs";
  const ::std::string logfile1 = base_name + ".part1.bfbs";
  // Remove it.
  unlink(config.c_str());
  unlink(logfile0.c_str());
  unlink(logfile1.c_str());

  LOG(INFO) << "Logging data to " << logfile0 << " and " << logfile1;

  {
    std::unique_ptr<EventLoop> logger_event_loop =
        event_loop_factory_.MakeEventLoop("logger");

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    Logger logger(logger_event_loop.get());
    logger.set_separate_config(false);
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

  EXPECT_THAT(reader.LoggedNodes(), ::testing::ElementsAre(nullptr));

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
  const ::std::string config =
      absl::StrCat(base_name, kSingleConfigSha256, ".bfbs");
  const ::std::string logfile = base_name + ".part0.bfbs";
  // Remove the log file.
  unlink(config.c_str());
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
    logger.set_separate_config(false);
    logger.set_polling_period(std::chrono::milliseconds(100));
    logger.StartLoggingLocalNamerOnRun(base_name);

    event_loop_factory_.RunFor(chrono::milliseconds(1000));
  }
}

// Parameters to run all the tests with.
struct Param {
  // The config file to use.
  std::string config;
  // If true, the RemoteMessage channel should be shared between all the remote
  // channels.  If false, there will be 1 RemoteMessage channel per remote
  // channel.
  bool shared;
  // sha256 of the config.
  std::string_view sha256;
};

class MultinodeLoggerTest : public ::testing::TestWithParam<struct Param> {
 public:
  MultinodeLoggerTest()
      : config_(aos::configuration::ReadConfig(ArtifactPath(
            absl::StrCat("aos/events/logging/", GetParam().config)))),
        time_converter_(configuration::NodesCount(&config_.message())),
        event_loop_factory_(&config_.message()),
        pi1_(event_loop_factory_.GetNodeEventLoopFactory("pi1")),
        pi1_index_(configuration::GetNodeIndex(
            event_loop_factory_.configuration(), pi1_->node())),
        pi2_(event_loop_factory_.GetNodeEventLoopFactory("pi2")),
        pi2_index_(configuration::GetNodeIndex(
            event_loop_factory_.configuration(), pi2_->node())),
        tmp_dir_(aos::testing::TestTmpDir()),
        logfile_base1_(tmp_dir_ + "/multi_logfile1"),
        logfile_base2_(tmp_dir_ + "/multi_logfile2"),
        pi1_reboot_logfiles_(MakePi1RebootLogfiles()),
        logfiles_(MakeLogFiles(logfile_base1_, logfile_base2_)),
        pi1_single_direction_logfiles_(MakePi1SingleDirectionLogfiles()),
        structured_logfiles_(StructureLogFiles()) {
    LOG(INFO) << "Config " << GetParam().config;
    event_loop_factory_.SetTimeConverter(&time_converter_);

    // Go through and remove the logfiles if they already exist.
    for (const auto &file : logfiles_) {
      unlink(file.c_str());
      unlink((file + ".xz").c_str());
    }

    for (const auto &file :
         MakeLogFiles(tmp_dir_ + "/relogged1", tmp_dir_ + "/relogged2")) {
      unlink(file.c_str());
    }

    for (const auto &file : pi1_reboot_logfiles_) {
      unlink(file.c_str());
    }

    LOG(INFO) << "Logging data to " << logfiles_[0] << ", " << logfiles_[1]
              << " and " << logfiles_[2];

    pi1_->OnStartup([this]() { pi1_->AlwaysStart<Ping>("ping"); });
    pi2_->OnStartup([this]() { pi2_->AlwaysStart<Pong>("pong"); });
  }

  bool shared() const { return GetParam().shared; }

  std::vector<std::string> MakeLogFiles(std::string logfile_base1,
                                        std::string logfile_base2,
                                        size_t pi1_data_count = 2,
                                        size_t pi2_data_count = 2) {
    std::vector<std::string> result;
    result.emplace_back(
        absl::StrCat(logfile_base1, "_", GetParam().sha256, ".bfbs"));
    result.emplace_back(
        absl::StrCat(logfile_base2, "_", GetParam().sha256, ".bfbs"));
    for (size_t i = 0; i < pi1_data_count; ++i) {
      result.emplace_back(
          absl::StrCat(logfile_base1, "_pi1_data.part", i, ".bfbs"));
    }
    result.emplace_back(logfile_base1 +
                        "_pi2_data/test/aos.examples.Pong.part0.bfbs");
    result.emplace_back(logfile_base1 +
                        "_pi2_data/test/aos.examples.Pong.part1.bfbs");
    for (size_t i = 0; i < pi2_data_count; ++i) {
      result.emplace_back(
          absl::StrCat(logfile_base2, "_pi2_data.part", i, ".bfbs"));
    }
    result.emplace_back(
        logfile_base2 +
        "_pi1_data/pi1/aos/aos.message_bridge.Timestamp.part0.bfbs");
    result.emplace_back(
        logfile_base2 +
        "_pi1_data/pi1/aos/aos.message_bridge.Timestamp.part1.bfbs");
    result.emplace_back(
        logfile_base1 +
        "_pi2_data/pi2/aos/aos.message_bridge.Timestamp.part0.bfbs");
    result.emplace_back(
        logfile_base1 +
        "_pi2_data/pi2/aos/aos.message_bridge.Timestamp.part1.bfbs");
    if (shared()) {
      result.emplace_back(logfile_base1 +
                          "_timestamps/pi1/aos/remote_timestamps/pi2/"
                          "aos.message_bridge.RemoteMessage.part0.bfbs");
      result.emplace_back(logfile_base1 +
                          "_timestamps/pi1/aos/remote_timestamps/pi2/"
                          "aos.message_bridge.RemoteMessage.part1.bfbs");
      result.emplace_back(logfile_base2 +
                          "_timestamps/pi2/aos/remote_timestamps/pi1/"
                          "aos.message_bridge.RemoteMessage.part0.bfbs");
      result.emplace_back(logfile_base2 +
                          "_timestamps/pi2/aos/remote_timestamps/pi1/"
                          "aos.message_bridge.RemoteMessage.part1.bfbs");
    } else {
      result.emplace_back(logfile_base1 +
                          "_timestamps/pi1/aos/remote_timestamps/pi2/pi1/aos/"
                          "aos-message_bridge-Timestamp/"
                          "aos.message_bridge.RemoteMessage.part0.bfbs");
      result.emplace_back(logfile_base1 +
                          "_timestamps/pi1/aos/remote_timestamps/pi2/pi1/aos/"
                          "aos-message_bridge-Timestamp/"
                          "aos.message_bridge.RemoteMessage.part1.bfbs");
      result.emplace_back(logfile_base2 +
                          "_timestamps/pi2/aos/remote_timestamps/pi1/pi2/aos/"
                          "aos-message_bridge-Timestamp/"
                          "aos.message_bridge.RemoteMessage.part0.bfbs");
      result.emplace_back(logfile_base2 +
                          "_timestamps/pi2/aos/remote_timestamps/pi1/pi2/aos/"
                          "aos-message_bridge-Timestamp/"
                          "aos.message_bridge.RemoteMessage.part1.bfbs");
      result.emplace_back(logfile_base1 +
                          "_timestamps/pi1/aos/remote_timestamps/pi2/test/"
                          "aos-examples-Ping/"
                          "aos.message_bridge.RemoteMessage.part0.bfbs");
      result.emplace_back(logfile_base1 +
                          "_timestamps/pi1/aos/remote_timestamps/pi2/test/"
                          "aos-examples-Ping/"
                          "aos.message_bridge.RemoteMessage.part1.bfbs");
    }

    return result;
  }

  std::vector<std::string> MakePi1RebootLogfiles() {
    std::vector<std::string> result;
    result.emplace_back(logfile_base1_ + "_pi1_data.part0.bfbs");
    result.emplace_back(logfile_base1_ + "_pi1_data.part1.bfbs");
    result.emplace_back(logfile_base1_ + "_pi1_data.part2.bfbs");
    result.emplace_back(logfile_base1_ +
                        "_pi2_data/test/aos.examples.Pong.part0.bfbs");
    result.emplace_back(logfile_base1_ +
                        "_pi2_data/test/aos.examples.Pong.part1.bfbs");
    result.emplace_back(logfile_base1_ +
                        "_pi2_data/test/aos.examples.Pong.part2.bfbs");
    result.emplace_back(logfile_base1_ +
                        "_pi2_data/test/aos.examples.Pong.part3.bfbs");
    result.emplace_back(
        logfile_base1_ +
        "_pi2_data/pi2/aos/aos.message_bridge.Timestamp.part0.bfbs");
    result.emplace_back(
        logfile_base1_ +
        "_pi2_data/pi2/aos/aos.message_bridge.Timestamp.part1.bfbs");
    result.emplace_back(
        logfile_base1_ +
        "_pi2_data/pi2/aos/aos.message_bridge.Timestamp.part2.bfbs");
    result.emplace_back(
        logfile_base1_ +
        "_pi2_data/pi2/aos/aos.message_bridge.Timestamp.part3.bfbs");
    result.emplace_back(
        absl::StrCat(logfile_base1_, "_", GetParam().sha256, ".bfbs"));
    if (shared()) {
      result.emplace_back(logfile_base1_ +
                          "_timestamps/pi1/aos/remote_timestamps/pi2/"
                          "aos.message_bridge.RemoteMessage.part0.bfbs");
      result.emplace_back(logfile_base1_ +
                          "_timestamps/pi1/aos/remote_timestamps/pi2/"
                          "aos.message_bridge.RemoteMessage.part1.bfbs");
      result.emplace_back(logfile_base1_ +
                          "_timestamps/pi1/aos/remote_timestamps/pi2/"
                          "aos.message_bridge.RemoteMessage.part2.bfbs");
      result.emplace_back(logfile_base1_ +
                          "_timestamps/pi1/aos/remote_timestamps/pi2/"
                          "aos.message_bridge.RemoteMessage.part3.bfbs");
    } else {
      result.emplace_back(logfile_base1_ +
                          "_timestamps/pi1/aos/remote_timestamps/pi2/pi1/aos/"
                          "aos-message_bridge-Timestamp/"
                          "aos.message_bridge.RemoteMessage.part0.bfbs");
      result.emplace_back(logfile_base1_ +
                          "_timestamps/pi1/aos/remote_timestamps/pi2/pi1/aos/"
                          "aos-message_bridge-Timestamp/"
                          "aos.message_bridge.RemoteMessage.part1.bfbs");
      result.emplace_back(logfile_base1_ +
                          "_timestamps/pi1/aos/remote_timestamps/pi2/pi1/aos/"
                          "aos-message_bridge-Timestamp/"
                          "aos.message_bridge.RemoteMessage.part2.bfbs");
      result.emplace_back(logfile_base1_ +
                          "_timestamps/pi1/aos/remote_timestamps/pi2/pi1/aos/"
                          "aos-message_bridge-Timestamp/"
                          "aos.message_bridge.RemoteMessage.part3.bfbs");

      result.emplace_back(logfile_base1_ +
                          "_timestamps/pi1/aos/remote_timestamps/pi2/test/"
                          "aos-examples-Ping/"
                          "aos.message_bridge.RemoteMessage.part0.bfbs");
      result.emplace_back(logfile_base1_ +
                          "_timestamps/pi1/aos/remote_timestamps/pi2/test/"
                          "aos-examples-Ping/"
                          "aos.message_bridge.RemoteMessage.part1.bfbs");
      result.emplace_back(logfile_base1_ +
                          "_timestamps/pi1/aos/remote_timestamps/pi2/test/"
                          "aos-examples-Ping/"
                          "aos.message_bridge.RemoteMessage.part2.bfbs");
      result.emplace_back(logfile_base1_ +
                          "_timestamps/pi1/aos/remote_timestamps/pi2/test/"
                          "aos-examples-Ping/"
                          "aos.message_bridge.RemoteMessage.part3.bfbs");
    }
    return result;
  }

  std::vector<std::string> MakePi1SingleDirectionLogfiles() {
    std::vector<std::string> result;
    result.emplace_back(logfile_base1_ + "_pi1_data.part0.bfbs");
    result.emplace_back(logfile_base1_ + "_pi1_data.part1.bfbs");
    result.emplace_back(
        logfile_base1_ +
        "_pi2_data/pi2/aos/aos.message_bridge.Timestamp.part0.bfbs");
    result.emplace_back(
        absl::StrCat(logfile_base1_, "_", GetParam().sha256, ".bfbs"));
    return result;
  }

  std::vector<std::string> MakePi1DeadNodeLogfiles() {
    std::vector<std::string> result;
    result.emplace_back(logfile_base1_ + "_pi1_data.part0.bfbs");
    result.emplace_back(
        absl::StrCat(logfile_base1_, "_", GetParam().sha256, ".bfbs"));
    return result;
  }

  std::vector<std::vector<std::string>> StructureLogFiles() {
    std::vector<std::vector<std::string>> result{
        std::vector<std::string>{logfiles_[2], logfiles_[3]},
        std::vector<std::string>{logfiles_[4], logfiles_[5]},
        std::vector<std::string>{logfiles_[6], logfiles_[7]},
        std::vector<std::string>{logfiles_[8], logfiles_[9]},
        std::vector<std::string>{logfiles_[10], logfiles_[11]},
        std::vector<std::string>{logfiles_[12], logfiles_[13]},
        std::vector<std::string>{logfiles_[14], logfiles_[15]}};

    if (!shared()) {
      result.emplace_back(
          std::vector<std::string>{logfiles_[16], logfiles_[17]});
    }

    return result;
  }

  struct LoggerState {
    std::unique_ptr<EventLoop> event_loop;
    std::unique_ptr<Logger> logger;
    const Configuration *configuration;
    const Node *node;
    MultiNodeLogNamer *log_namer;
  };

  LoggerState MakeLogger(NodeEventLoopFactory *node,
                         SimulatedEventLoopFactory *factory = nullptr,
                         const Configuration *configuration = nullptr) {
    if (factory == nullptr) {
      factory = &event_loop_factory_;
    }
    if (configuration == nullptr) {
      configuration = factory->configuration();
    }
    return {node->MakeEventLoop("logger"),
            {},
            configuration,
            configuration::GetNode(configuration, node->node()),
            nullptr};
  }

  void StartLogger(LoggerState *logger, std::string logfile_base = "",
                   bool compress = false) {
    if (logfile_base.empty()) {
      if (logger->event_loop->node()->name()->string_view() == "pi1") {
        logfile_base = logfile_base1_;
      } else {
        logfile_base = logfile_base2_;
      }
    }

    logger->logger = std::make_unique<Logger>(logger->event_loop.get(),
                                              logger->configuration);
    logger->logger->set_polling_period(std::chrono::milliseconds(100));
    logger->logger->set_name(absl::StrCat(
        "name_prefix_", logger->event_loop->node()->name()->str()));
    logger->event_loop->OnRun([logger, logfile_base, compress]() {
      std::unique_ptr<MultiNodeLogNamer> namer =
          std::make_unique<MultiNodeLogNamer>(
              logfile_base, logger->configuration, logger->event_loop.get(),
              logger->node);
      if (compress) {
#ifdef LZMA
        namer->set_extension(".xz");
        namer->set_encoder_factory(
            []() { return std::make_unique<aos::logger::LzmaEncoder>(3); });
#else
        LOG(FATAL) << "Compression unsupported";
#endif
      }
      logger->log_namer = namer.get();

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
      EXPECT_TRUE(log_file.config);
      EXPECT_EQ(log_file.name,
                absl::StrCat("name_prefix_", log_file.logger_node));

      for (const LogParts &part : log_file.parts) {
        EXPECT_NE(part.monotonic_start_time, aos::monotonic_clock::min_time)
            << ": " << part;
        missing_rt_count +=
            part.realtime_start_time == aos::realtime_clock::min_time;

        EXPECT_TRUE(log_event_uuids.find(part.log_event_uuid) !=
                    log_event_uuids.end());
        EXPECT_NE(part.node, "");
        EXPECT_TRUE(log_file.config);
        parts_uuids.insert(part.parts_uuid);
        both_uuids.insert(part.parts_uuid);
      }
    }

    // We won't have RT timestamps for 5 or 6 log files.  We don't log the RT
    // start time on remote nodes because we don't know it and would be
    // guessing.  And the log reader can actually do a better job.  The number
    // depends on if we have the remote timestamps split across 2 files, or just
    // across 1, depending on if we are using a split or combined timestamp
    // channel config.
    EXPECT_EQ(missing_rt_count, shared() ? 5u : 6u);

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
  message_bridge::TestingTimeConverter time_converter_;
  SimulatedEventLoopFactory event_loop_factory_;

  NodeEventLoopFactory *const pi1_;
  const size_t pi1_index_;
  NodeEventLoopFactory *const pi2_;
  const size_t pi2_index_;

  std::string tmp_dir_;
  std::string logfile_base1_;
  std::string logfile_base2_;
  std::vector<std::string> pi1_reboot_logfiles_;
  std::vector<std::string> logfiles_;
  std::vector<std::string> pi1_single_direction_logfiles_;

  std::vector<std::vector<std::string>> structured_logfiles_;
};

// Counts the number of messages on a channel.  Returns (channel name, channel
// type, count) for every message matching matcher()
std::vector<std::tuple<std::string, std::string, int>> CountChannelsMatching(
    std::shared_ptr<const aos::Configuration> config, std::string_view filename,
    std::function<bool(const MessageHeader *)> matcher) {
  MessageReader message_reader(filename);
  std::vector<int> counts(config->channels()->size(), 0);

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
      const Channel *channel = config->channels()->Get(i);
      result.push_back(std::make_tuple(channel->name()->str(),
                                       channel->type()->str(), counts[i]));
    }
    ++channel;
  }

  return result;
}

// Counts the number of messages (channel, count) for all data messages.
std::vector<std::tuple<std::string, std::string, int>> CountChannelsData(
    std::shared_ptr<const aos::Configuration> config,
    std::string_view filename) {
  return CountChannelsMatching(config, filename, [](const MessageHeader *msg) {
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
    std::shared_ptr<const aos::Configuration> config,
    std::string_view filename) {
  return CountChannelsMatching(config, filename, [](const MessageHeader *msg) {
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
TEST_P(MultinodeLoggerTest, SimpleMultiNode) {
  time_converter_.StartEqual();
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
      if (!log_header.back().message().has_configuration()) {
        logfile_uuids.insert(
            log_header.back().message().log_event_uuid()->str());
        parts_uuids.insert(log_header.back().message().parts_uuid()->str());
      }
    }

    EXPECT_EQ(logfile_uuids.size(), 2u);
    if (shared()) {
      EXPECT_EQ(parts_uuids.size(), 7u);
    } else {
      EXPECT_EQ(parts_uuids.size(), 8u);
    }

    // And confirm everything is on the correct node.
    EXPECT_EQ(log_header[2].message().node()->name()->string_view(), "pi1");
    EXPECT_EQ(log_header[3].message().node()->name()->string_view(), "pi1");
    EXPECT_EQ(log_header[4].message().node()->name()->string_view(), "pi2");
    EXPECT_EQ(log_header[5].message().node()->name()->string_view(), "pi2");
    EXPECT_EQ(log_header[6].message().node()->name()->string_view(), "pi2");
    EXPECT_EQ(log_header[7].message().node()->name()->string_view(), "pi2");
    EXPECT_EQ(log_header[8].message().node()->name()->string_view(), "pi1");
    EXPECT_EQ(log_header[9].message().node()->name()->string_view(), "pi1");
    EXPECT_EQ(log_header[10].message().node()->name()->string_view(), "pi2");
    EXPECT_EQ(log_header[11].message().node()->name()->string_view(), "pi2");
    EXPECT_EQ(log_header[12].message().node()->name()->string_view(), "pi2");
    EXPECT_EQ(log_header[13].message().node()->name()->string_view(), "pi2");
    EXPECT_EQ(log_header[14].message().node()->name()->string_view(), "pi1");
    EXPECT_EQ(log_header[15].message().node()->name()->string_view(), "pi1");
    if (!shared()) {
      EXPECT_EQ(log_header[16].message().node()->name()->string_view(), "pi2");
      EXPECT_EQ(log_header[17].message().node()->name()->string_view(), "pi2");
    }

    // And the parts index matches.
    EXPECT_EQ(log_header[2].message().parts_index(), 0);
    EXPECT_EQ(log_header[3].message().parts_index(), 1);
    EXPECT_EQ(log_header[4].message().parts_index(), 0);
    EXPECT_EQ(log_header[5].message().parts_index(), 1);
    EXPECT_EQ(log_header[6].message().parts_index(), 0);
    EXPECT_EQ(log_header[7].message().parts_index(), 1);
    EXPECT_EQ(log_header[8].message().parts_index(), 0);
    EXPECT_EQ(log_header[9].message().parts_index(), 1);
    EXPECT_EQ(log_header[10].message().parts_index(), 0);
    EXPECT_EQ(log_header[11].message().parts_index(), 1);
    EXPECT_EQ(log_header[12].message().parts_index(), 0);
    EXPECT_EQ(log_header[13].message().parts_index(), 1);
    EXPECT_EQ(log_header[14].message().parts_index(), 0);
    EXPECT_EQ(log_header[15].message().parts_index(), 1);
    if (!shared()) {
      EXPECT_EQ(log_header[16].message().parts_index(), 0);
      EXPECT_EQ(log_header[17].message().parts_index(), 1);
    }
  }

  const std::vector<LogFile> sorted_log_files = SortParts(logfiles_);
  {
    using ::testing::UnorderedElementsAre;
    std::shared_ptr<const aos::Configuration> config =
        sorted_log_files[0].config;

    // Timing reports, pings
    EXPECT_THAT(
        CountChannelsData(config, logfiles_[2]),
        UnorderedElementsAre(
            std::make_tuple("/pi1/aos", "aos.message_bridge.ServerStatistics",
                            1),
            std::make_tuple("/test", "aos.examples.Ping", 1)))
        << " : " << logfiles_[2];
    EXPECT_THAT(
        CountChannelsData(config, logfiles_[3]),
        UnorderedElementsAre(
            std::make_tuple("/pi1/aos", "aos.message_bridge.Timestamp", 200),
            std::make_tuple("/pi1/aos", "aos.message_bridge.ServerStatistics",
                            20),
            std::make_tuple("/pi1/aos", "aos.message_bridge.ClientStatistics",
                            200),
            std::make_tuple("/pi1/aos", "aos.timing.Report", 40),
            std::make_tuple("/test", "aos.examples.Ping", 2000)))
        << " : " << logfiles_[3];
    // Timestamps for pong
    EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[2]),
                UnorderedElementsAre())
        << " : " << logfiles_[2];
    EXPECT_THAT(
        CountChannelsTimestamp(config, logfiles_[3]),
        UnorderedElementsAre(
            std::make_tuple("/test", "aos.examples.Pong", 2001),
            std::make_tuple("/pi2/aos", "aos.message_bridge.Timestamp", 200)))
        << " : " << logfiles_[3];

    // Pong data.
    EXPECT_THAT(
        CountChannelsData(config, logfiles_[4]),
        UnorderedElementsAre(std::make_tuple("/test", "aos.examples.Pong", 91)))
        << " : " << logfiles_[4];
    EXPECT_THAT(CountChannelsData(config, logfiles_[5]),
                UnorderedElementsAre(
                    std::make_tuple("/test", "aos.examples.Pong", 1910)))
        << " : " << logfiles_[5];

    // No timestamps
    EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[4]),
                UnorderedElementsAre())
        << " : " << logfiles_[4];
    EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[5]),
                UnorderedElementsAre())
        << " : " << logfiles_[5];

    // Timing reports and pongs.
    EXPECT_THAT(CountChannelsData(config, logfiles_[6]),
                UnorderedElementsAre(std::make_tuple(
                    "/pi2/aos", "aos.message_bridge.ServerStatistics", 1)))
        << " : " << logfiles_[6];
    EXPECT_THAT(
        CountChannelsData(config, logfiles_[7]),
        UnorderedElementsAre(
            std::make_tuple("/pi2/aos", "aos.message_bridge.Timestamp", 200),
            std::make_tuple("/pi2/aos", "aos.message_bridge.ServerStatistics",
                            20),
            std::make_tuple("/pi2/aos", "aos.message_bridge.ClientStatistics",
                            200),
            std::make_tuple("/pi2/aos", "aos.timing.Report", 40),
            std::make_tuple("/test", "aos.examples.Pong", 2001)))
        << " : " << logfiles_[7];
    // And ping timestamps.
    EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[6]),
                UnorderedElementsAre())
        << " : " << logfiles_[6];
    EXPECT_THAT(
        CountChannelsTimestamp(config, logfiles_[7]),
        UnorderedElementsAre(
            std::make_tuple("/test", "aos.examples.Ping", 2001),
            std::make_tuple("/pi1/aos", "aos.message_bridge.Timestamp", 200)))
        << " : " << logfiles_[7];

    // And then test that the remotely logged timestamp data files only have
    // timestamps in them.
    EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[8]),
                UnorderedElementsAre())
        << " : " << logfiles_[8];
    EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[9]),
                UnorderedElementsAre())
        << " : " << logfiles_[9];
    EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[10]),
                UnorderedElementsAre())
        << " : " << logfiles_[10];
    EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[11]),
                UnorderedElementsAre())
        << " : " << logfiles_[11];

    EXPECT_THAT(CountChannelsData(config, logfiles_[8]),
                UnorderedElementsAre(std::make_tuple(
                    "/pi1/aos", "aos.message_bridge.Timestamp", 9)))
        << " : " << logfiles_[8];
    EXPECT_THAT(CountChannelsData(config, logfiles_[9]),
                UnorderedElementsAre(std::make_tuple(
                    "/pi1/aos", "aos.message_bridge.Timestamp", 191)))
        << " : " << logfiles_[9];

    EXPECT_THAT(CountChannelsData(config, logfiles_[10]),
                UnorderedElementsAre(std::make_tuple(
                    "/pi2/aos", "aos.message_bridge.Timestamp", 9)))
        << " : " << logfiles_[10];
    EXPECT_THAT(CountChannelsData(config, logfiles_[11]),
                UnorderedElementsAre(std::make_tuple(
                    "/pi2/aos", "aos.message_bridge.Timestamp", 191)))
        << " : " << logfiles_[11];

    // Timestamps from pi2 on pi1, and the other way.
    EXPECT_THAT(CountChannelsData(config, logfiles_[12]),
                UnorderedElementsAre())
        << " : " << logfiles_[12];
    EXPECT_THAT(CountChannelsData(config, logfiles_[13]),
                UnorderedElementsAre())
        << " : " << logfiles_[13];
    EXPECT_THAT(CountChannelsData(config, logfiles_[14]),
                UnorderedElementsAre())
        << " : " << logfiles_[14];
    EXPECT_THAT(CountChannelsData(config, logfiles_[15]),
                UnorderedElementsAre())
        << " : " << logfiles_[15];
    if (!shared()) {
      EXPECT_THAT(CountChannelsData(config, logfiles_[16]),
                  UnorderedElementsAre())
          << " : " << logfiles_[16];
      EXPECT_THAT(CountChannelsData(config, logfiles_[17]),
                  UnorderedElementsAre())
          << " : " << logfiles_[17];
    }

    if (shared()) {
      EXPECT_THAT(
          CountChannelsTimestamp(config, logfiles_[12]),
          UnorderedElementsAre(
              std::make_tuple("/pi1/aos", "aos.message_bridge.Timestamp", 9),
              std::make_tuple("/test", "aos.examples.Ping", 91)))
          << " : " << logfiles_[12];
      EXPECT_THAT(
          CountChannelsTimestamp(config, logfiles_[13]),
          UnorderedElementsAre(
              std::make_tuple("/pi1/aos", "aos.message_bridge.Timestamp", 191),
              std::make_tuple("/test", "aos.examples.Ping", 1910)))
          << " : " << logfiles_[13];
      EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[14]),
                  UnorderedElementsAre(std::make_tuple(
                      "/pi2/aos", "aos.message_bridge.Timestamp", 9)))
          << " : " << logfiles_[14];
      EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[15]),
                  UnorderedElementsAre(std::make_tuple(
                      "/pi2/aos", "aos.message_bridge.Timestamp", 191)))
          << " : " << logfiles_[15];
    } else {
      EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[12]),
                  UnorderedElementsAre(std::make_tuple(
                      "/pi1/aos", "aos.message_bridge.Timestamp", 9)))
          << " : " << logfiles_[12];
      EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[13]),
                  UnorderedElementsAre(std::make_tuple(
                      "/pi1/aos", "aos.message_bridge.Timestamp", 191)))
          << " : " << logfiles_[13];
      EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[14]),
                  UnorderedElementsAre(std::make_tuple(
                      "/pi2/aos", "aos.message_bridge.Timestamp", 9)))
          << " : " << logfiles_[14];
      EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[15]),
                  UnorderedElementsAre(std::make_tuple(
                      "/pi2/aos", "aos.message_bridge.Timestamp", 191)))
          << " : " << logfiles_[15];
      EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[16]),
                  UnorderedElementsAre(
                      std::make_tuple("/test", "aos.examples.Ping", 91)))
          << " : " << logfiles_[16];
      EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[17]),
                  UnorderedElementsAre(
                      std::make_tuple("/test", "aos.examples.Ping", 1910)))
          << " : " << logfiles_[17];
    }
  }

  LogReader reader(sorted_log_files);

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

  EXPECT_THAT(reader.LoggedNodes(),
              ::testing::ElementsAre(
                  configuration::GetNode(reader.logged_configuration(), pi1),
                  configuration::GetNode(reader.logged_configuration(), pi2)));

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
TEST_P(MultinodeLoggerDeathTest, MultiNodeBadReplayConfig) {
  time_converter_.StartEqual();
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
TEST_P(MultinodeLoggerTest, StaggeredStart) {
  time_converter_.StartEqual();
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(200));

    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }

  // Since we delay starting pi2, it already knows about all the timestamps so
  // we don't end up with extra parts.
  LogReader reader(
      SortParts(MakeLogFiles(logfile_base1_, logfile_base2_, 2, 1)));

  SimulatedEventLoopFactory log_reader_factory(reader.configuration());
  log_reader_factory.set_send_delay(chrono::microseconds(0));

  // This sends out the fetched messages and advances time to the start of the
  // log file.
  reader.Register(&log_reader_factory);

  const Node *pi1 =
      configuration::GetNode(log_reader_factory.configuration(), "pi1");
  const Node *pi2 =
      configuration::GetNode(log_reader_factory.configuration(), "pi2");

  EXPECT_THAT(reader.LoggedNodes(),
              ::testing::ElementsAre(
                  configuration::GetNode(reader.logged_configuration(), pi1),
                  configuration::GetNode(reader.logged_configuration(), pi2)));

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
TEST_P(MultinodeLoggerTest, MismatchedClocks) {
  // TODO(austin): Negate...
  const chrono::nanoseconds initial_pi2_offset = chrono::seconds(1000);

  time_converter_.AddMonotonic(
      {BootTimestamp::epoch(), BootTimestamp::epoch() + initial_pi2_offset});
  // Wait for 95 ms, (~0.1 seconds - 1/2 of the ping/pong period), and set the
  // skew to be 200 uS/s
  const chrono::nanoseconds startup_sleep1 = time_converter_.AddMonotonic(
      {chrono::milliseconds(95),
       chrono::milliseconds(95) - chrono::nanoseconds(200) * 95});
  // Run another 200 ms to have one logger start first.
  const chrono::nanoseconds startup_sleep2 = time_converter_.AddMonotonic(
      {chrono::milliseconds(200), chrono::milliseconds(200)});
  // Slew one way then the other at the same 200 uS/S slew rate.  Make sure we
  // go far enough to cause problems if this isn't accounted for.
  const chrono::nanoseconds logger_run1 = time_converter_.AddMonotonic(
      {chrono::milliseconds(20000),
       chrono::milliseconds(20000) - chrono::nanoseconds(200) * 20000});
  const chrono::nanoseconds logger_run2 = time_converter_.AddMonotonic(
      {chrono::milliseconds(40000),
       chrono::milliseconds(40000) + chrono::nanoseconds(200) * 40000});
  const chrono::nanoseconds logger_run3 = time_converter_.AddMonotonic(
      {chrono::milliseconds(400), chrono::milliseconds(400)});

  {
    LoggerState pi2_logger = MakeLogger(pi2_);

    LOG(INFO) << "pi2 times: " << pi2_->monotonic_now() << " "
              << pi2_->realtime_now() << " distributed "
              << pi2_->ToDistributedClock(pi2_->monotonic_now());

    LOG(INFO) << "pi2_ times: " << pi2_->monotonic_now() << " "
              << pi2_->realtime_now() << " distributed "
              << pi2_->ToDistributedClock(pi2_->monotonic_now());

    event_loop_factory_.RunFor(startup_sleep1);

    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(startup_sleep2);

    {
      // Run pi1's logger for only part of the time.
      LoggerState pi1_logger = MakeLogger(pi1_);

      StartLogger(&pi1_logger);
      event_loop_factory_.RunFor(logger_run1);

      // Make sure we slewed time far enough so that the difference is greater
      // than the network delay.  This confirms that if we sort incorrectly, it
      // would show in the results.
      EXPECT_LT(
          (pi2_->monotonic_now() - pi1_->monotonic_now()) - initial_pi2_offset,
          -event_loop_factory_.send_delay() -
              event_loop_factory_.network_delay());

      event_loop_factory_.RunFor(logger_run2);

      // And now check that we went far enough the other way to make sure we
      // cover both problems.
      EXPECT_GT(
          (pi2_->monotonic_now() - pi1_->monotonic_now()) - initial_pi2_offset,
          event_loop_factory_.send_delay() +
              event_loop_factory_.network_delay());
    }

    // And log a bit more on pi2.
    event_loop_factory_.RunFor(logger_run3);
  }

  LogReader reader(
      SortParts(MakeLogFiles(logfile_base1_, logfile_base2_, 3, 2)));

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

  EXPECT_THAT(reader.LoggedNodes(),
              ::testing::ElementsAre(
                  configuration::GetNode(reader.logged_configuration(), pi1),
                  configuration::GetNode(reader.logged_configuration(), pi2)));

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
TEST_P(MultinodeLoggerTest, SortParts) {
  time_converter_.StartEqual();
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
TEST_P(MultinodeLoggerTest, SortEmptyParts) {
  time_converter_.StartEqual();
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
TEST_P(MultinodeLoggerTest, SortEmptyCompressedParts) {
  time_converter_.StartEqual();
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
TEST_P(MultinodeLoggerTest, SortTruncatedCompressedParts) {
  time_converter_.StartEqual();
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
      aos::util::ReadFileToStringOrDie(logfiles_[2]);

  aos::util::WriteStringToFileOrDie(
      logfiles_[2],
      compressed_contents.substr(0, compressed_contents.size() - 100));

  const std::vector<LogFile> sorted_parts = SortParts(logfiles_);
  VerifyParts(sorted_parts);
}
#endif

// Tests that if we remap a remapped channel, it shows up correctly.
TEST_P(MultinodeLoggerTest, RemapLoggedChannel) {
  time_converter_.StartEqual();
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

// Tests that we can remap a forwarded channel as well.
TEST_P(MultinodeLoggerTest, RemapForwardedLoggedChannel) {
  time_converter_.StartEqual();
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }

  LogReader reader(SortParts(logfiles_));

  reader.RemapLoggedChannel<examples::Ping>("/test");

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

  MessageCounter<examples::Ping> pi1_ping(pi1_event_loop.get(), "/test");
  MessageCounter<examples::Ping> pi2_ping(pi2_event_loop.get(), "/test");
  MessageCounter<examples::Ping> pi1_original_ping(pi1_event_loop.get(),
                                                   "/original/test");
  MessageCounter<examples::Ping> pi2_original_ping(pi2_event_loop.get(),
                                                   "/original/test");

  std::unique_ptr<MessageCounter<message_bridge::RemoteMessage>>
      pi1_original_ping_timestamp;
  std::unique_ptr<MessageCounter<message_bridge::RemoteMessage>>
      pi1_ping_timestamp;
  if (!shared()) {
    pi1_original_ping_timestamp =
        std::make_unique<MessageCounter<message_bridge::RemoteMessage>>(
            pi1_event_loop.get(),
            "/pi1/aos/remote_timestamps/pi2/original/test/aos-examples-Ping");
    pi1_ping_timestamp =
        std::make_unique<MessageCounter<message_bridge::RemoteMessage>>(
            pi1_event_loop.get(),
            "/pi1/aos/remote_timestamps/pi2/test/aos-examples-Ping");
  }

  log_reader_factory.Run();

  EXPECT_EQ(pi1_ping.count(), 0u);
  EXPECT_EQ(pi2_ping.count(), 0u);
  EXPECT_NE(pi1_original_ping.count(), 0u);
  EXPECT_NE(pi2_original_ping.count(), 0u);
  if (!shared()) {
    EXPECT_NE(pi1_original_ping_timestamp->count(), 0u);
    EXPECT_EQ(pi1_ping_timestamp->count(), 0u);
  }

  reader.Deregister();
}

// Tests that we properly recreate forwarded timestamps when replaying a log.
// This should be enough that we can then re-run the logger and get a valid log
// back.
TEST_P(MultinodeLoggerTest, MessageHeader) {
  time_converter_.StartEqual();
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

  EXPECT_THAT(reader.LoggedNodes(),
              ::testing::ElementsAre(
                  configuration::GetNode(reader.logged_configuration(), pi1),
                  configuration::GetNode(reader.logged_configuration(), pi2)));

  reader.event_loop_factory()->set_send_delay(chrono::microseconds(0));

  std::unique_ptr<EventLoop> pi1_event_loop =
      log_reader_factory.MakeEventLoop("test", pi1);
  std::unique_ptr<EventLoop> pi2_event_loop =
      log_reader_factory.MakeEventLoop("test", pi2);

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

  const chrono::nanoseconds network_delay = event_loop_factory_.network_delay();
  const chrono::nanoseconds send_delay = event_loop_factory_.send_delay();

  for (std::pair<int, std::string> channel :
       shared()
           ? std::vector<
                 std::pair<int, std::string>>{{-1,
                                               "/aos/remote_timestamps/pi2"}}
           : std::vector<std::pair<int, std::string>>{
                 {pi1_timestamp_channel,
                  "/aos/remote_timestamps/pi2/pi1/aos/"
                  "aos-message_bridge-Timestamp"},
                 {ping_timestamp_channel,
                  "/aos/remote_timestamps/pi2/test/aos-examples-Ping"}}) {
    pi1_event_loop->MakeWatcher(
        channel.second,
        [&pi1_event_loop, &pi2_event_loop, pi1_timestamp_channel,
         ping_timestamp_channel, &pi1_timestamp_on_pi1_fetcher,
         &pi1_timestamp_on_pi2_fetcher, &ping_on_pi1_fetcher,
         &ping_on_pi2_fetcher, network_delay, send_delay,
         channel_index = channel.first](const RemoteMessage &header) {
          const aos::monotonic_clock::time_point header_monotonic_sent_time(
              chrono::nanoseconds(header.monotonic_sent_time()));
          const aos::realtime_clock::time_point header_realtime_sent_time(
              chrono::nanoseconds(header.realtime_sent_time()));
          const aos::monotonic_clock::time_point header_monotonic_remote_time(
              chrono::nanoseconds(header.monotonic_remote_time()));
          const aos::realtime_clock::time_point header_realtime_remote_time(
              chrono::nanoseconds(header.realtime_remote_time()));

          if (channel_index != -1) {
            ASSERT_EQ(channel_index, header.channel_index());
          }

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
          EXPECT_EQ(UUID::FromVector(header.boot_uuid()),
                    pi2_event_loop->boot_uuid());

          EXPECT_EQ(pi1_context->queue_index, header.remote_queue_index());
          EXPECT_EQ(pi2_context->remote_queue_index,
                    header.remote_queue_index());
          EXPECT_EQ(pi2_context->queue_index, header.queue_index());

          EXPECT_EQ(pi2_context->monotonic_event_time,
                    header_monotonic_sent_time);
          EXPECT_EQ(pi2_context->realtime_event_time,
                    header_realtime_sent_time);
          EXPECT_EQ(pi2_context->realtime_remote_time,
                    header_realtime_remote_time);
          EXPECT_EQ(pi2_context->monotonic_remote_time,
                    header_monotonic_remote_time);

          EXPECT_EQ(pi1_context->realtime_event_time,
                    header_realtime_remote_time);
          EXPECT_EQ(pi1_context->monotonic_event_time,
                    header_monotonic_remote_time);

          // Time estimation isn't perfect, but we know the clocks were
          // identical when logged, so we know when this should have come back.
          // Confirm we got it when we expected.
          EXPECT_EQ(pi1_event_loop->context().monotonic_event_time,
                    pi1_context->monotonic_event_time + 2 * network_delay +
                        send_delay);
        });
  }
  for (std::pair<int, std::string> channel :
       shared()
           ? std::vector<
                 std::pair<int, std::string>>{{-1,
                                               "/aos/remote_timestamps/pi1"}}
           : std::vector<std::pair<int, std::string>>{
                 {pi2_timestamp_channel,
                  "/aos/remote_timestamps/pi1/pi2/aos/"
                  "aos-message_bridge-Timestamp"}}) {
    pi2_event_loop->MakeWatcher(
        channel.second,
        [&pi2_event_loop, &pi1_event_loop, pi2_timestamp_channel,
         pong_timestamp_channel, &pi2_timestamp_on_pi2_fetcher,
         &pi2_timestamp_on_pi1_fetcher, &pong_on_pi2_fetcher,
         &pong_on_pi1_fetcher, network_delay, send_delay,
         channel_index = channel.first](const RemoteMessage &header) {
          const aos::monotonic_clock::time_point header_monotonic_sent_time(
              chrono::nanoseconds(header.monotonic_sent_time()));
          const aos::realtime_clock::time_point header_realtime_sent_time(
              chrono::nanoseconds(header.realtime_sent_time()));
          const aos::monotonic_clock::time_point header_monotonic_remote_time(
              chrono::nanoseconds(header.monotonic_remote_time()));
          const aos::realtime_clock::time_point header_realtime_remote_time(
              chrono::nanoseconds(header.realtime_remote_time()));

          if (channel_index != -1) {
            ASSERT_EQ(channel_index, header.channel_index());
          }

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
          EXPECT_EQ(UUID::FromVector(header.boot_uuid()),
                    pi1_event_loop->boot_uuid());

          EXPECT_EQ(pi2_context->queue_index, header.remote_queue_index());
          EXPECT_EQ(pi1_context->remote_queue_index,
                    header.remote_queue_index());
          EXPECT_EQ(pi1_context->queue_index, header.queue_index());

          EXPECT_EQ(pi1_context->monotonic_event_time,
                    header_monotonic_sent_time);
          EXPECT_EQ(pi1_context->realtime_event_time,
                    header_realtime_sent_time);
          EXPECT_EQ(pi1_context->realtime_remote_time,
                    header_realtime_remote_time);
          EXPECT_EQ(pi1_context->monotonic_remote_time,
                    header_monotonic_remote_time);

          EXPECT_EQ(pi2_context->realtime_event_time,
                    header_realtime_remote_time);
          EXPECT_EQ(pi2_context->monotonic_event_time,
                    header_monotonic_remote_time);

          // Time estimation isn't perfect, but we know the clocks were
          // identical when logged, so we know when this should have come back.
          // Confirm we got it when we expected.
          EXPECT_EQ(pi2_event_loop->context().monotonic_event_time,
                    pi2_context->monotonic_event_time + 2 * network_delay +
                        send_delay);
        });
  }

  // And confirm we can re-create a log again, while checking the contents.
  {
    LoggerState pi1_logger = MakeLogger(
        log_reader_factory.GetNodeEventLoopFactory("pi1"), &log_reader_factory);
    LoggerState pi2_logger = MakeLogger(
        log_reader_factory.GetNodeEventLoopFactory("pi2"), &log_reader_factory);

    StartLogger(&pi1_logger, tmp_dir_ + "/relogged1");
    StartLogger(&pi2_logger, tmp_dir_ + "/relogged2");

    log_reader_factory.Run();
  }

  reader.Deregister();

  // And verify that we can run the LogReader over the relogged files without
  // hitting any fatal errors.
  {
    LogReader relogged_reader(SortParts(
        MakeLogFiles(tmp_dir_ + "/relogged1", tmp_dir_ + "/relogged2")));
    relogged_reader.Register();

    relogged_reader.event_loop_factory()->Run();
  }
}

// Tests that we properly populate and extract the logger_start time by setting
// up a clock difference between 2 nodes and looking at the resulting parts.
TEST_P(MultinodeLoggerTest, LoggerStartTime) {
  time_converter_.AddMonotonic(
      {BootTimestamp::epoch(),
       BootTimestamp::epoch() + chrono::seconds(1000)});
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

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

// Test that renaming the base, renames the folder.
TEST_F(MultinodeLoggerTest, LoggerRenameFolder) {
  util::UnlinkRecursive(tmp_dir_ + "/renamefolder");
  util::UnlinkRecursive(tmp_dir_ + "/new-good");
  time_converter_.AddMonotonic(
      {BootTimestamp::epoch(),
       BootTimestamp::epoch() + chrono::seconds(1000)});
  logfile_base1_ = tmp_dir_ + "/renamefolder/multi_logfile1";
  logfile_base2_ = tmp_dir_ + "/renamefolder/multi_logfile2";
  logfiles_ = MakeLogFiles(logfile_base1_, logfile_base2_);
  LoggerState pi1_logger = MakeLogger(pi1_);
  LoggerState pi2_logger = MakeLogger(pi2_);

  StartLogger(&pi1_logger);
  StartLogger(&pi2_logger);

  event_loop_factory_.RunFor(chrono::milliseconds(10000));
  logfile_base1_ = tmp_dir_ + "/new-good/multi_logfile1";
  logfile_base2_ = tmp_dir_ + "/new-good/multi_logfile2";
  logfiles_ = MakeLogFiles(logfile_base1_, logfile_base2_);
  ASSERT_TRUE(pi1_logger.logger->RenameLogBase(logfile_base1_));
  ASSERT_TRUE(pi2_logger.logger->RenameLogBase(logfile_base2_));
  for (auto &file : logfiles_) {
    struct stat s;
    EXPECT_EQ(0, stat(file.c_str(), &s));
  }
}

// Test that renaming the file base dies.
TEST_P(MultinodeLoggerDeathTest, LoggerRenameFile) {
  time_converter_.AddMonotonic(
      {BootTimestamp::epoch(), BootTimestamp::epoch() + chrono::seconds(1000)});
  util::UnlinkRecursive(tmp_dir_ + "/renamefile");
  logfile_base1_ = tmp_dir_ + "/renamefile/multi_logfile1";
  logfile_base2_ = tmp_dir_ + "/renamefile/multi_logfile2";
  logfiles_ = MakeLogFiles(logfile_base1_, logfile_base2_);
  LoggerState pi1_logger = MakeLogger(pi1_);
  StartLogger(&pi1_logger);
  event_loop_factory_.RunFor(chrono::milliseconds(10000));
  logfile_base1_ = tmp_dir_ + "/new-renamefile/new_multi_logfile1";
  EXPECT_DEATH({ pi1_logger.logger->RenameLogBase(logfile_base1_); },
               "Rename of file base from");
}

// TODO(austin): We can write a test which recreates a logfile and confirms that
// we get it back.  That is the ultimate test.

// Tests that we properly recreate forwarded timestamps when replaying a log.
// This should be enough that we can then re-run the logger and get a valid log
// back.
TEST_P(MultinodeLoggerTest, RemoteReboot) {
  const UUID pi1_boot0 = UUID::Random();
  const UUID pi2_boot0 = UUID::Random();
  const UUID pi2_boot1 = UUID::Random();
  {
    CHECK_EQ(pi1_index_, 0u);
    CHECK_EQ(pi2_index_, 1u);

    time_converter_.set_boot_uuid(pi1_index_, 0, pi1_boot0);
    time_converter_.set_boot_uuid(pi2_index_, 0, pi2_boot0);
    time_converter_.set_boot_uuid(pi2_index_, 1, pi2_boot1);

    time_converter_.AddNextTimestamp(
        distributed_clock::epoch(),
        {BootTimestamp::epoch(), BootTimestamp::epoch()});
    const chrono::nanoseconds reboot_time = chrono::milliseconds(10100);
    time_converter_.AddNextTimestamp(
        distributed_clock::epoch() + reboot_time,
        {BootTimestamp::epoch() + reboot_time,
         BootTimestamp{
             .boot = 1,
             .time = monotonic_clock::epoch() + chrono::milliseconds(1323)}});
  }

  {
    LoggerState pi1_logger = MakeLogger(pi1_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));
    EXPECT_EQ(event_loop_factory_.GetNodeEventLoopFactory("pi1")->boot_uuid(),
              pi1_boot0);
    EXPECT_EQ(event_loop_factory_.GetNodeEventLoopFactory("pi2")->boot_uuid(),
              pi2_boot0);

    StartLogger(&pi1_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(10000));

    VLOG(1) << "Reboot now!";

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
    EXPECT_EQ(event_loop_factory_.GetNodeEventLoopFactory("pi1")->boot_uuid(),
              pi1_boot0);
    EXPECT_EQ(event_loop_factory_.GetNodeEventLoopFactory("pi2")->boot_uuid(),
              pi2_boot1);
  }

  // Confirm that our new oldest timestamps properly update as we reboot and
  // rotate.
  for (const std::string &file : pi1_reboot_logfiles_) {
    std::optional<SizePrefixedFlatbufferVector<LogFileHeader>> log_header =
        ReadHeader(file);
    CHECK(log_header);
    if (log_header->message().has_configuration()) {
      continue;
    }

    const monotonic_clock::time_point monotonic_start_time =
        monotonic_clock::time_point(
            chrono::nanoseconds(log_header->message().monotonic_start_time()));
    const UUID source_node_boot_uuid = UUID::FromString(
        log_header->message().source_node_boot_uuid()->string_view());

    if (log_header->message().node()->name()->string_view() != "pi1") {
      switch (log_header->message().parts_index()) {
        case 0:
          EXPECT_EQ(source_node_boot_uuid, pi2_boot0);
          EXPECT_EQ(monotonic_start_time, monotonic_clock::min_time);
          break;
        case 1:
          EXPECT_EQ(source_node_boot_uuid, pi2_boot0);
          ASSERT_EQ(monotonic_start_time,
                    monotonic_clock::epoch() + chrono::seconds(1));
          break;
        case 2:
          EXPECT_EQ(source_node_boot_uuid, pi2_boot1);
          EXPECT_EQ(monotonic_start_time, monotonic_clock::min_time);
          break;
        case 3:
          EXPECT_EQ(source_node_boot_uuid, pi2_boot1);
          ASSERT_EQ(monotonic_start_time,
                    monotonic_clock::epoch() + chrono::nanoseconds(2322999462));
          break;
        default:
          FAIL();
          break;
      }
      continue;
    }
    SCOPED_TRACE(file);
    SCOPED_TRACE(aos::FlatbufferToJson(
        *log_header, {.multi_line = true, .max_vector_size = 100}));
    ASSERT_TRUE(log_header->message().has_oldest_remote_monotonic_timestamps());
    ASSERT_EQ(
        log_header->message().oldest_remote_monotonic_timestamps()->size(), 2u);
    EXPECT_EQ(
        log_header->message().oldest_remote_monotonic_timestamps()->Get(0),
        monotonic_clock::max_time.time_since_epoch().count());
    ASSERT_TRUE(log_header->message().has_oldest_local_monotonic_timestamps());
    ASSERT_EQ(log_header->message().oldest_local_monotonic_timestamps()->size(),
              2u);
    EXPECT_EQ(log_header->message().oldest_local_monotonic_timestamps()->Get(0),
              monotonic_clock::max_time.time_since_epoch().count());
    ASSERT_TRUE(log_header->message()
                    .has_oldest_remote_unreliable_monotonic_timestamps());
    ASSERT_EQ(log_header->message()
                  .oldest_remote_unreliable_monotonic_timestamps()
                  ->size(),
              2u);
    EXPECT_EQ(log_header->message()
                  .oldest_remote_unreliable_monotonic_timestamps()
                  ->Get(0),
              monotonic_clock::max_time.time_since_epoch().count());
    ASSERT_TRUE(log_header->message()
                    .has_oldest_local_unreliable_monotonic_timestamps());
    ASSERT_EQ(log_header->message()
                  .oldest_local_unreliable_monotonic_timestamps()
                  ->size(),
              2u);
    EXPECT_EQ(log_header->message()
                  .oldest_local_unreliable_monotonic_timestamps()
                  ->Get(0),
              monotonic_clock::max_time.time_since_epoch().count());

    const monotonic_clock::time_point oldest_remote_monotonic_timestamps =
        monotonic_clock::time_point(chrono::nanoseconds(
            log_header->message().oldest_remote_monotonic_timestamps()->Get(
                1)));
    const monotonic_clock::time_point oldest_local_monotonic_timestamps =
        monotonic_clock::time_point(chrono::nanoseconds(
            log_header->message().oldest_local_monotonic_timestamps()->Get(1)));
    const monotonic_clock::time_point
        oldest_remote_unreliable_monotonic_timestamps =
            monotonic_clock::time_point(chrono::nanoseconds(
                log_header->message()
                    .oldest_remote_unreliable_monotonic_timestamps()
                    ->Get(1)));
    const monotonic_clock::time_point
        oldest_local_unreliable_monotonic_timestamps =
            monotonic_clock::time_point(chrono::nanoseconds(
                log_header->message()
                    .oldest_local_unreliable_monotonic_timestamps()
                    ->Get(1)));
    switch (log_header->message().parts_index()) {
      case 0:
        EXPECT_EQ(oldest_remote_monotonic_timestamps,
                  monotonic_clock::max_time);
        EXPECT_EQ(oldest_local_monotonic_timestamps, monotonic_clock::max_time);
        EXPECT_EQ(oldest_remote_unreliable_monotonic_timestamps,
                  monotonic_clock::max_time);
        EXPECT_EQ(oldest_local_unreliable_monotonic_timestamps,
                  monotonic_clock::max_time);
        break;
      case 1:
        EXPECT_EQ(oldest_remote_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(90200)));
        EXPECT_EQ(oldest_local_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(90350)));
        EXPECT_EQ(oldest_remote_unreliable_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(90200)));
        EXPECT_EQ(oldest_local_unreliable_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(90350)));
        break;
      case 2:
        EXPECT_EQ(oldest_remote_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::milliseconds(1323) +
                                              chrono::microseconds(200)));
        EXPECT_EQ(oldest_local_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(10100350)));
        EXPECT_EQ(oldest_remote_unreliable_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::milliseconds(1323) +
                                              chrono::microseconds(200)));
        EXPECT_EQ(oldest_local_unreliable_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(10100350)));
        break;
      default:
        FAIL();
        break;
    }
  }

  // Confirm that we refuse to replay logs with missing boot uuids.
  {
    LogReader reader(SortParts(pi1_reboot_logfiles_));

    SimulatedEventLoopFactory log_reader_factory(reader.configuration());
    log_reader_factory.set_send_delay(chrono::microseconds(0));

    // This sends out the fetched messages and advances time to the start of
    // the log file.
    reader.Register(&log_reader_factory);

    log_reader_factory.Run();

    reader.Deregister();
  }
}

// Tests that we properly handle one direction of message_bridge being
// unavailable.
TEST_P(MultinodeLoggerTest, OneDirectionWithNegativeSlope) {
  pi1_->Disconnect(pi2_->node());
  time_converter_.AddMonotonic(
      {BootTimestamp::epoch(),
       BootTimestamp::epoch() + chrono::seconds(1000)});

  time_converter_.AddMonotonic(
      {chrono::milliseconds(10000),
       chrono::milliseconds(10000) - chrono::milliseconds(1)});
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
TEST_P(MultinodeLoggerTest, OneDirectionWithPositiveSlope) {
  pi1_->Disconnect(pi2_->node());
  time_converter_.AddMonotonic(
      {BootTimestamp::epoch(),
       BootTimestamp::epoch() + chrono::seconds(500)});

  time_converter_.AddMonotonic(
      {chrono::milliseconds(10000),
       chrono::milliseconds(10000) + chrono::milliseconds(1)});
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

// Tests that we properly handle a dead node.  Do this by just disconnecting it
// and only using one nodes of logs.
TEST_P(MultinodeLoggerTest, DeadNode) {
  pi1_->Disconnect(pi2_->node());
  pi2_->Disconnect(pi1_->node());
  time_converter_.AddMonotonic(
      {BootTimestamp::epoch(),
       BootTimestamp::epoch() + chrono::seconds(1000)});
  {
    LoggerState pi1_logger = MakeLogger(pi1_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(10000));
  }

  // Confirm that we can parse the result.  LogReader has enough internal CHECKs
  // to confirm the right thing happened.
  ConfirmReadable(MakePi1DeadNodeLogfiles());
}

constexpr std::string_view kCombinedConfigSha1(
    "4503751edc96327493562f0376f0d6daac172927c0fd64d04ce5d67505186c0b");
constexpr std::string_view kSplitConfigSha1(
    "918a748432c5e70a971dfd8934968378bed04ab61cf2efcd35b7f6224053c247");

INSTANTIATE_TEST_SUITE_P(
    All, MultinodeLoggerTest,
    ::testing::Values(Param{"multinode_pingpong_combined_config.json", true,
                            kCombinedConfigSha1},
                      Param{"multinode_pingpong_split_config.json", false,
                            kSplitConfigSha1}));

INSTANTIATE_TEST_SUITE_P(
    All, MultinodeLoggerDeathTest,
    ::testing::Values(Param{"multinode_pingpong_combined_config.json", true,
                            kCombinedConfigSha1},
                      Param{"multinode_pingpong_split_config.json", false,
                            kSplitConfigSha1}));

// Tests that we can relog with a different config.  This makes most sense when
// you are trying to edit a log and want to use channel renaming + the original
// config in the new log.
TEST_P(MultinodeLoggerTest, LogDifferentConfig) {
  time_converter_.StartEqual();
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }

  LogReader reader(SortParts(logfiles_));
  reader.RemapLoggedChannel<aos::examples::Ping>("/test", "/original");

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

  EXPECT_THAT(reader.LoggedNodes(),
              ::testing::ElementsAre(
                  configuration::GetNode(reader.logged_configuration(), pi1),
                  configuration::GetNode(reader.logged_configuration(), pi2)));

  reader.event_loop_factory()->set_send_delay(chrono::microseconds(0));

  // And confirm we can re-create a log again, while checking the contents.
  std::vector<std::string> log_files;
  {
    LoggerState pi1_logger =
        MakeLogger(log_reader_factory.GetNodeEventLoopFactory("pi1"),
                   &log_reader_factory, reader.logged_configuration());
    LoggerState pi2_logger =
        MakeLogger(log_reader_factory.GetNodeEventLoopFactory("pi2"),
                   &log_reader_factory, reader.logged_configuration());

    StartLogger(&pi1_logger, tmp_dir_ + "/relogged1");
    StartLogger(&pi2_logger, tmp_dir_ + "/relogged2");

    log_reader_factory.Run();

    for (auto &x : pi1_logger.log_namer->all_filenames()) {
      log_files.emplace_back(absl::StrCat(tmp_dir_, "/relogged1_", x));
    }
    for (auto &x : pi2_logger.log_namer->all_filenames()) {
      log_files.emplace_back(absl::StrCat(tmp_dir_, "/relogged2_", x));
    }
  }

  reader.Deregister();

  // And verify that we can run the LogReader over the relogged files without
  // hitting any fatal errors.
  {
    LogReader relogged_reader(SortParts(log_files));
    relogged_reader.Register();

    relogged_reader.event_loop_factory()->Run();
  }
}

}  // namespace testing
}  // namespace logger
}  // namespace aos
