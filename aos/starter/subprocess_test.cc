#include "aos/starter/subprocess.h"

#include "aos/events/shm_event_loop.h"
#include "aos/testing/path.h"
#include "aos/testing/tmpdir.h"
#include "aos/util/file.h"
#include "gtest/gtest.h"

namespace aos::starter::testing {

class SubprocessTest : public ::testing::Test {
 protected:
  SubprocessTest() : shm_dir_(aos::testing::TestTmpDir() + "/aos") {
    FLAGS_shm_base = shm_dir_;

    // Nuke the shm dir:
    aos::util::UnlinkRecursive(shm_dir_);
  }

  gflags::FlagSaver flag_saver_;
  std::string shm_dir_;
};

TEST_F(SubprocessTest, CaptureOutputs) {
  const std::string config_file =
      ::aos::testing::ArtifactPath("aos/events/pingpong_config.json");

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(config_file);
  aos::ShmEventLoop event_loop(&config.message());
  bool observed_stopped = false;
  Application echo_stdout(
      "echo", "echo", &event_loop, [&observed_stopped, &echo_stdout]() {
        if (echo_stdout.status() == aos::starter::State::STOPPED) {
          observed_stopped = true;
        }
      });
  ASSERT_FALSE(echo_stdout.autorestart());
  echo_stdout.set_args({"abcdef"});
  echo_stdout.set_capture_stdout(true);
  echo_stdout.set_capture_stderr(true);

  echo_stdout.Start();
  aos::TimerHandler *exit_timer =
      event_loop.AddTimer([&event_loop]() { event_loop.Exit(); });
  event_loop.OnRun([&event_loop, exit_timer]() {
    exit_timer->Setup(event_loop.monotonic_now() + std::chrono::seconds(1));
  });

  event_loop.Run();

  ASSERT_EQ("abcdef\n", echo_stdout.GetStdout());
  ASSERT_TRUE(echo_stdout.GetStderr().empty());
  EXPECT_TRUE(observed_stopped);
  EXPECT_EQ(aos::starter::State::STOPPED, echo_stdout.status());

  observed_stopped = false;

  // Run again, the output should've been cleared.
  echo_stdout.set_args({"ghijkl"});
  echo_stdout.Start();
  event_loop.Run();
  ASSERT_EQ("ghijkl\n", echo_stdout.GetStdout());
  EXPECT_TRUE(observed_stopped);
}

TEST_F(SubprocessTest, CaptureStderr) {
  const std::string config_file =
      ::aos::testing::ArtifactPath("aos/events/pingpong_config.json");

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(config_file);
  aos::ShmEventLoop event_loop(&config.message());
  bool observed_stopped = false;
  Application echo_stderr(
      "echo", "sh", &event_loop, [&observed_stopped, &echo_stderr]() {
        if (echo_stderr.status() == aos::starter::State::STOPPED) {
          observed_stopped = true;
        }
      });
  echo_stderr.set_args({"-c", "echo abcdef >&2"});
  echo_stderr.set_capture_stdout(true);
  echo_stderr.set_capture_stderr(true);

  echo_stderr.Start();
  event_loop.AddTimer([&event_loop]() { event_loop.Exit(); })
      ->Setup(event_loop.monotonic_now() + std::chrono::seconds(1));

  event_loop.Run();

  ASSERT_EQ("abcdef\n", echo_stderr.GetStderr());
  ASSERT_TRUE(echo_stderr.GetStdout().empty());
  ASSERT_TRUE(observed_stopped);
  ASSERT_EQ(aos::starter::State::STOPPED, echo_stderr.status());
}

}  // namespace aos::starter::testing
