#include "gtest/gtest.h"

#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/ping_lib.h"
#include "aos/events/pong_lib.h"
#include "aos/events/shm_event_loop.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/path.h"
#include "aos/testing/tmpdir.h"

DECLARE_string(override_hostname);

namespace aos::logger::testing {

class RealtimeLoggerTest : public ::testing::Test {
 protected:
  RealtimeLoggerTest()
      : shm_dir_(aos::testing::TestTmpDir() + "/aos"),
        config_file_(
            aos::testing::ArtifactPath("aos/events/pingpong_config.json")),
        config_(aos::configuration::ReadConfig(config_file_)),
        event_loop_factory_(&config_.message()),
        ping_event_loop_(event_loop_factory_.MakeEventLoop("ping")),
        pong_event_loop_(event_loop_factory_.MakeEventLoop("pong")),
        ping_(ping_event_loop_.get()),
        pong_(pong_event_loop_.get()),
        tmpdir_(aos::testing::TestTmpDir()),
        base_name_(tmpdir_ + "/logfile/") {
    FLAGS_shm_base = shm_dir_;

    // Nuke the shm and log dirs, to ensure we aren't being affected by any
    // preexisting tests.
    aos::util::UnlinkRecursive(shm_dir_);
    aos::util::UnlinkRecursive(base_name_);
  }

  gflags::FlagSaver flag_saver_;
  std::string shm_dir_;

  const std::string config_file_;
  const aos::FlatbufferDetachedBuffer<aos::Configuration> config_;

  // Factory and Ping class to generate a test logfile.
  SimulatedEventLoopFactory event_loop_factory_;
  std::unique_ptr<EventLoop> ping_event_loop_;
  std::unique_ptr<EventLoop> pong_event_loop_;
  Ping ping_;
  Pong pong_;
  const std::string tmpdir_;
  const std::string base_name_;
};

class RealtimeMultiNodeLoggerTest : public ::testing::Test {
 protected:
  RealtimeMultiNodeLoggerTest()
      : shm_dir_(aos::testing::TestTmpDir() + "/aos"),
        config_file_(aos::testing::ArtifactPath(
            "aos/events/logging/multinode_pingpong_combined_config.json")),
        config_(aos::configuration::ReadConfig(config_file_)),
        event_loop_factory_(&config_.message()),
        ping_event_loop_(event_loop_factory_.MakeEventLoop(
            "pi1", configuration::GetNode(&config_.message(), "pi1"))),
        ping_(ping_event_loop_.get()),
        tmpdir_(aos::testing::TestTmpDir()),
        base_name_(tmpdir_ + "/logfile/") {
    FLAGS_shm_base = shm_dir_;

    // Nuke the shm and log dirs, to ensure we aren't being affected by any
    // preexisting tests.
    aos::util::UnlinkRecursive(shm_dir_);
    aos::util::UnlinkRecursive(base_name_);
  }

  gflags::FlagSaver flag_saver_;
  std::string shm_dir_;

  const std::string config_file_;
  const aos::FlatbufferDetachedBuffer<aos::Configuration> config_;

  // Factory and Ping class to generate a test logfile.
  SimulatedEventLoopFactory event_loop_factory_;
  std::unique_ptr<EventLoop> ping_event_loop_;
  Ping ping_;
  const std::string tmpdir_;
  const std::string base_name_;
};

TEST_F(RealtimeLoggerTest, RealtimeReplay) {
  {
    std::unique_ptr<EventLoop> logger_event_loop =
        event_loop_factory_.MakeEventLoop("logger");

    event_loop_factory_.RunFor(std::chrono::milliseconds(95));

    Logger logger(logger_event_loop.get());
    logger.set_separate_config(false);
    logger.set_polling_period(std::chrono::milliseconds(100));
    logger.StartLoggingOnRun(base_name_);
    event_loop_factory_.RunFor(std::chrono::milliseconds(2000));
  }

  LogReader reader(logger::SortParts(logger::FindLogs(base_name_)));
  ShmEventLoop shm_event_loop(reader.configuration());
  reader.Register(&shm_event_loop);
  reader.OnEnd(shm_event_loop.node(),
               [&shm_event_loop]() { shm_event_loop.Exit(); });

  Fetcher<examples::Ping> ping_fetcher =
      shm_event_loop.MakeFetcher<examples::Ping>("/test");

  shm_event_loop.AddTimer([]() { LOG(INFO) << "Hello, World!"; })
      ->Schedule(shm_event_loop.monotonic_now(), std::chrono::seconds(1));

  shm_event_loop.Run();
  reader.Deregister();

  ASSERT_TRUE(ping_fetcher.Fetch());
  ASSERT_EQ(ping_fetcher->value(), 210);
}

// Tests that ReplayChannels causes no messages to be replayed other than what
// is included on a single node config
TEST_F(RealtimeLoggerTest, SingleNodeReplayChannels) {
  {
    std::unique_ptr<EventLoop> logger_event_loop =
        event_loop_factory_.MakeEventLoop("logger");

    event_loop_factory_.RunFor(std::chrono::milliseconds(95));

    Logger logger(logger_event_loop.get());
    logger.set_separate_config(false);
    logger.set_polling_period(std::chrono::milliseconds(100));
    logger.StartLoggingOnRun(base_name_);
    event_loop_factory_.RunFor(std::chrono::milliseconds(2000));
  }

  ReplayChannels replay_channels{{"/test", "aos.examples.Ping"}};
  LogReader reader(logger::SortParts(logger::FindLogs(base_name_)),
                   &config_.message(), &replay_channels);
  ShmEventLoop shm_event_loop(reader.configuration());
  reader.Register(&shm_event_loop);
  reader.OnEnd(shm_event_loop.node(),
               [&shm_event_loop]() { shm_event_loop.Exit(); });

  Fetcher<examples::Ping> ping_fetcher =
      shm_event_loop.MakeFetcher<examples::Ping>("/test");
  Fetcher<examples::Pong> pong_fetcher =
      shm_event_loop.MakeFetcher<examples::Pong>("/test");

  shm_event_loop.AddTimer([]() { LOG(INFO) << "Hello, World!"; })
      ->Schedule(shm_event_loop.monotonic_now(), std::chrono::seconds(1));

  // End timer should not be called in this case, it should automatically quit
  // the event loop and check for number of fetches messages
  // This is added to make sure OnEnd is called consistently
  // When OnEnd is not called after finishing of the log, this will eventually
  // quit due to the end timer but will report a failure
  size_t run_seconds = 3;
  auto *const end_timer =
      shm_event_loop.AddTimer([&shm_event_loop, run_seconds]() {
        shm_event_loop.Exit();
        FAIL() << "OnEnd wasn't called on log end so quitting after "
               << run_seconds << " seconds.";
      });
  shm_event_loop.OnRun([&shm_event_loop, end_timer, run_seconds]() {
    LOG(INFO) << "Quitting in: " << run_seconds;
    end_timer->Schedule(shm_event_loop.monotonic_now() +
                        std::chrono::seconds(run_seconds));
  });

  shm_event_loop.Run();
  reader.Deregister();

  ASSERT_TRUE(ping_fetcher.Fetch());
  ASSERT_EQ(ping_fetcher->value(), 210);
  ASSERT_FALSE(pong_fetcher.Fetch());
}

// Tests that ReplayChannels causes no messages to be replayed other than what
// is included on a multi node config
TEST_F(RealtimeMultiNodeLoggerTest, ReplayChannelsPingTest) {
  FLAGS_override_hostname = "raspberrypi";
  {
    std::unique_ptr<EventLoop> logger_event_loop =
        event_loop_factory_.MakeEventLoop(
            "logger", configuration::GetNode(&config_.message(), "pi1"));

    event_loop_factory_.RunFor(std::chrono::milliseconds(95));

    Logger logger(logger_event_loop.get());
    logger.set_separate_config(false);
    logger.set_polling_period(std::chrono::milliseconds(100));

    std::unique_ptr<MultiNodeFilesLogNamer> namer =
        std::make_unique<MultiNodeFilesLogNamer>(
            base_name_, &config_.message(), logger_event_loop.get(),
            configuration::GetNode(&config_.message(), "pi1"));

    logger.StartLogging(std::move(namer));
    event_loop_factory_.RunFor(std::chrono::milliseconds(2000));
  }

  ReplayChannels replay_channels{{"/test", "aos.examples.Ping"}};
  LogReader reader(logger::SortParts(logger::FindLogs(base_name_)),
                   &config_.message(), &replay_channels);
  ShmEventLoop shm_event_loop(reader.configuration());
  reader.Register(&shm_event_loop);
  reader.OnEnd(shm_event_loop.node(),
               [&shm_event_loop]() { shm_event_loop.Exit(); });

  Fetcher<examples::Ping> ping_fetcher =
      shm_event_loop.MakeFetcher<examples::Ping>("/test");

  shm_event_loop.AddTimer([]() { LOG(INFO) << "Hello, World!"; })
      ->Schedule(shm_event_loop.monotonic_now(), std::chrono::seconds(1));

  shm_event_loop.Run();
  reader.Deregister();

  ASSERT_TRUE(ping_fetcher.Fetch());
  ASSERT_EQ(ping_fetcher->value(), 210);
}

// Tests that when remapping a channel included in ReplayChannels messages are
// sent on the remapped channel
TEST_F(RealtimeMultiNodeLoggerTest, RemappedReplayChannelsTest) {
  FLAGS_override_hostname = "raspberrypi";
  {
    std::unique_ptr<EventLoop> logger_event_loop =
        event_loop_factory_.MakeEventLoop(
            "logger", configuration::GetNode(&config_.message(), "pi1"));

    event_loop_factory_.RunFor(std::chrono::milliseconds(95));

    Logger logger(logger_event_loop.get());
    logger.set_separate_config(false);
    logger.set_polling_period(std::chrono::milliseconds(100));

    std::unique_ptr<MultiNodeFilesLogNamer> namer =
        std::make_unique<MultiNodeFilesLogNamer>(
            base_name_, &config_.message(), logger_event_loop.get(),
            configuration::GetNode(&config_.message(), "pi1"));

    logger.StartLogging(std::move(namer));
    event_loop_factory_.RunFor(std::chrono::milliseconds(2000));
  }

  ReplayChannels replay_channels{{"/test", "aos.examples.Ping"}};
  LogReader reader(logger::SortParts(logger::FindLogs(base_name_)),
                   &config_.message(), &replay_channels);
  reader.RemapLoggedChannel<aos::examples::Ping>("/test", "/original");
  ShmEventLoop shm_event_loop(reader.configuration());
  reader.Register(&shm_event_loop);
  reader.OnEnd(shm_event_loop.node(),
               [&shm_event_loop]() { shm_event_loop.Exit(); });

  Fetcher<examples::Ping> original_ping_fetcher =
      shm_event_loop.MakeFetcher<examples::Ping>("/original/test");

  Fetcher<examples::Ping> ping_fetcher =
      shm_event_loop.MakeFetcher<examples::Ping>("/test");

  shm_event_loop.AddTimer([]() { LOG(INFO) << "Hello, World!"; })
      ->Schedule(shm_event_loop.monotonic_now(), std::chrono::seconds(1));

  shm_event_loop.Run();
  reader.Deregister();

  ASSERT_TRUE(original_ping_fetcher.Fetch());
  ASSERT_EQ(original_ping_fetcher->value(), 210);
  ASSERT_FALSE(ping_fetcher.Fetch());
}

// Tests that messages are not replayed when they do not exist in the
// ReplayChannels provided to LogReader. The channels used here do not
// exist in the log being replayed, and there's no messages on those
// channels as well.
TEST_F(RealtimeMultiNodeLoggerTest, DoesNotExistInReplayChannelsTest) {
  FLAGS_override_hostname = "raspberrypi";
  {
    std::unique_ptr<EventLoop> logger_event_loop =
        event_loop_factory_.MakeEventLoop(
            "logger", configuration::GetNode(&config_.message(), "pi1"));

    event_loop_factory_.RunFor(std::chrono::milliseconds(95));

    Logger logger(logger_event_loop.get());
    logger.set_separate_config(false);
    logger.set_polling_period(std::chrono::milliseconds(100));
    std::unique_ptr<MultiNodeFilesLogNamer> namer =
        std::make_unique<MultiNodeFilesLogNamer>(
            base_name_, &config_.message(), logger_event_loop.get(),
            configuration::GetNode(&config_.message(), "pi1"));

    logger.StartLogging(std::move(namer));
    event_loop_factory_.RunFor(std::chrono::milliseconds(2000));
  }

  ReplayChannels replay_channels{{"/test", "aos.examples.Pong"},
                                 {"/test", "fake"},
                                 {"fake", "aos.examples.Ping"}};
  LogReader reader(logger::SortParts(logger::FindLogs(base_name_)),
                   &config_.message(), &replay_channels);
  ShmEventLoop shm_event_loop(reader.configuration());
  reader.Register(&shm_event_loop);
  reader.OnEnd(shm_event_loop.node(),
               [&shm_event_loop]() { shm_event_loop.Exit(); });

  Fetcher<examples::Ping> ping_fetcher =
      shm_event_loop.MakeFetcher<examples::Ping>("/test");

  auto *const end_timer = shm_event_loop.AddTimer([&shm_event_loop]() {
    LOG(INFO) << "All done, quitting now";
    shm_event_loop.Exit();
  });

  // TODO(#21) reader.OnEnd() is not working as expected when
  // using replay_channels
  // keep looking for 3 seconds if some message comes, just in case
  size_t run_seconds = 3;
  shm_event_loop.OnRun([&shm_event_loop, end_timer, run_seconds]() {
    LOG(INFO) << "Quitting in: " << run_seconds;
    end_timer->Schedule(shm_event_loop.monotonic_now() +
                        std::chrono::seconds(run_seconds));
  });

  shm_event_loop.Run();
  reader.Deregister();
  ASSERT_FALSE(ping_fetcher.Fetch());
}

using RealtimeMultiNodeLoggerDeathTest = RealtimeMultiNodeLoggerTest;

// Tests that remapping a channel not included in the replay channels passed to
// LogReader throws an error since this would indicate the user is trying to use
// the channel being remapped.
TEST_F(RealtimeMultiNodeLoggerDeathTest,
       RemapLoggedChannelNotIncludedInReplayChannels) {
  FLAGS_override_hostname = "raspberrypi";
  {
    std::unique_ptr<EventLoop> logger_event_loop =
        event_loop_factory_.MakeEventLoop(
            "logger", configuration::GetNode(&config_.message(), "pi1"));

    event_loop_factory_.RunFor(std::chrono::milliseconds(95));

    Logger logger(logger_event_loop.get());
    logger.set_separate_config(false);
    logger.set_polling_period(std::chrono::milliseconds(100));

    std::unique_ptr<MultiNodeFilesLogNamer> namer =
        std::make_unique<MultiNodeFilesLogNamer>(
            base_name_, &config_.message(), logger_event_loop.get(),
            configuration::GetNode(&config_.message(), "pi1"));

    logger.StartLogging(std::move(namer));
    event_loop_factory_.RunFor(std::chrono::milliseconds(2000));
  }

  ReplayChannels replay_channels{{"/test", "aos.examples.Ping"}};
  LogReader reader(logger::SortParts(logger::FindLogs(base_name_)),
                   &config_.message(), &replay_channels);
  EXPECT_DEATH(
      reader.RemapLoggedChannel<aos::examples::Ping>("/fake", "/original"),
      "which is not included in the replay channels passed to LogReader");
}

}  // namespace aos::logger::testing
