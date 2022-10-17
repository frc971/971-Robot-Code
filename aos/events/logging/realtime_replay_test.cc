#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/ping_lib.h"
#include "aos/events/shm_event_loop.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/path.h"
#include "aos/testing/tmpdir.h"
#include "gtest/gtest.h"

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
        ping_(ping_event_loop_.get()) {
    FLAGS_shm_base = shm_dir_;

    // Nuke the shm dir, to ensure we aren't being affected by any preexisting
    // tests.
    aos::util::UnlinkRecursive(shm_dir_);
  }

  gflags::FlagSaver flag_saver_;
  std::string shm_dir_;

  const std::string config_file_;
  const aos::FlatbufferDetachedBuffer<aos::Configuration> config_;

  // Factory and Ping class to generate a test logfile.
  SimulatedEventLoopFactory event_loop_factory_;
  std::unique_ptr<EventLoop> ping_event_loop_;
  Ping ping_;
};

TEST_F(RealtimeLoggerTest, RealtimeReplay) {
  const std::string tmpdir = aos::testing::TestTmpDir();
  const std::string base_name = tmpdir + "/logfile/";
  aos::util::UnlinkRecursive(base_name);
  {
    std::unique_ptr<EventLoop> logger_event_loop =
        event_loop_factory_.MakeEventLoop("logger");

    event_loop_factory_.RunFor(std::chrono::milliseconds(95));

    Logger logger(logger_event_loop.get());
    logger.set_separate_config(false);
    logger.set_polling_period(std::chrono::milliseconds(100));
    logger.StartLoggingOnRun(base_name);
    event_loop_factory_.RunFor(std::chrono::milliseconds(2000));
  }

  LogReader reader(logger::SortParts(logger::FindLogs(base_name)));
  ShmEventLoop shm_event_loop(reader.configuration());
  reader.Register(&shm_event_loop);
  reader.OnEnd(shm_event_loop.node(),
               [&shm_event_loop]() { shm_event_loop.Exit(); });

  Fetcher<examples::Ping> ping_fetcher =
      shm_event_loop.MakeFetcher<examples::Ping>("/test");

  shm_event_loop.AddTimer([]() { LOG(INFO) << "Hello, World!"; })
      ->Setup(shm_event_loop.monotonic_now(), std::chrono::seconds(1));

  shm_event_loop.Run();
  reader.Deregister();

  ASSERT_TRUE(ping_fetcher.Fetch());
  ASSERT_EQ(ping_fetcher->value(), 210);
}
}  // namespace aos::logger::testing
