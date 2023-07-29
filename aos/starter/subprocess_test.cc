#include "aos/starter/subprocess.h"

#include <signal.h>
#include <sys/types.h>

#include "absl/strings/str_join.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/events/shm_event_loop.h"
#include "aos/testing/path.h"
#include "aos/testing/tmpdir.h"
#include "aos/util/file.h"

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
    // Note: we are using the backup poll in this test to capture SIGCHLD.  This
    // runs at 1 hz, so make sure we let it run at least once.
    exit_timer->Schedule(event_loop.monotonic_now() +
                         std::chrono::milliseconds(1500));
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
  // Note: we are using the backup poll in this test to capture SIGCHLD.  This
  // runs at 1 hz, so make sure we let it run at least once.
  event_loop.AddTimer([&event_loop]() { event_loop.Exit(); })
      ->Schedule(event_loop.monotonic_now() + std::chrono::milliseconds(1500));

  event_loop.Run();

  ASSERT_EQ("abcdef\n", echo_stderr.GetStderr());
  ASSERT_TRUE(echo_stderr.GetStdout().empty());
  ASSERT_TRUE(observed_stopped);
  ASSERT_EQ(aos::starter::State::STOPPED, echo_stderr.status());
}

TEST_F(SubprocessTest, UnactiveQuietFlag) {
  const std::string config_file =
      ::aos::testing::ArtifactPath("aos/events/pingpong_config.json");

  ::testing::internal::CaptureStderr();

  // Set up application without quiet flag active
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(config_file);
  aos::ShmEventLoop event_loop(&config.message());
  bool observed_stopped = false;
  Application error_out(
      "false", "false", &event_loop,
      [&observed_stopped, &error_out]() {
        if (error_out.status() == aos::starter::State::STOPPED) {
          observed_stopped = true;
        }
      },
      Application::QuietLogging::kNo);
  ASSERT_FALSE(error_out.autorestart());

  error_out.Start();
  aos::TimerHandler *exit_timer =
      event_loop.AddTimer([&event_loop]() { event_loop.Exit(); });
  event_loop.OnRun([&event_loop, exit_timer]() {
    exit_timer->Schedule(event_loop.monotonic_now() +
                         std::chrono::milliseconds(1500));
  });

  event_loop.Run();

  // Ensure presence of logs without quiet flag
  std::string output = ::testing::internal::GetCapturedStderr();
  std::string expectedStart = "Failed to start 'false'";
  std::string expectedRun = "exited unexpectedly with status";

  ASSERT_TRUE(output.find(expectedStart) != std::string::npos ||
              output.find(expectedRun) != std::string::npos);
  EXPECT_TRUE(observed_stopped);
  EXPECT_EQ(aos::starter::State::STOPPED, error_out.status());
}

TEST_F(SubprocessTest, ActiveQuietFlag) {
  const std::string config_file =
      ::aos::testing::ArtifactPath("aos/events/pingpong_config.json");

  ::testing::internal::CaptureStderr();

  // Set up application with quiet flag active
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(config_file);
  aos::ShmEventLoop event_loop(&config.message());
  bool observed_stopped = false;
  Application error_out(
      "false", "false", &event_loop,
      [&observed_stopped, &error_out]() {
        if (error_out.status() == aos::starter::State::STOPPED) {
          observed_stopped = true;
        }
      },
      Application::QuietLogging::kYes);
  ASSERT_FALSE(error_out.autorestart());

  error_out.Start();
  aos::TimerHandler *exit_timer =
      event_loop.AddTimer([&event_loop]() { event_loop.Exit(); });
  event_loop.OnRun([&event_loop, exit_timer]() {
    exit_timer->Schedule(event_loop.monotonic_now() +
                         std::chrono::milliseconds(1500));
  });

  event_loop.Run();

  // Ensure lack of logs with quiet flag
  ASSERT_TRUE(::testing::internal::GetCapturedStderr().empty());
  EXPECT_TRUE(observed_stopped);
  EXPECT_EQ(aos::starter::State::STOPPED, error_out.status());
}

// Tests that Nothing Bad™ happens if the event loop outlives the Application.
//
// Note that this is a bit of a hope test, as there is no guarantee that we
// will trigger a crash even if the resources tied to the event loop in the
// aos::Application aren't properly released.
TEST_F(SubprocessTest, ShortLivedApp) {
  const std::string config_file =
      ::aos::testing::ArtifactPath("aos/events/pingpong_config.json");

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(config_file);
  aos::ShmEventLoop event_loop(&config.message());

  auto application =
      std::make_unique<Application>("sleep", "sleep", &event_loop, []() {});
  application->set_args({"10"});
  application->Start();
  pid_t pid = application->get_pid();

  int ticks = 0;
  aos::TimerHandler *exit_timer = event_loop.AddTimer([&event_loop, &ticks,
                                                       &application, pid]() {
    ticks++;
    if (application && application->status() == aos::starter::State::RUNNING) {
      // Kill the application, it will autorestart.
      kill(pid, SIGTERM);
      application.reset();
    }

    // event loop lives for longer.
    if (ticks >= 5) {
      // Now we exit.
      event_loop.Exit();
    }
  });

  event_loop.OnRun([&event_loop, exit_timer]() {
    exit_timer->Schedule(event_loop.monotonic_now(),
                         std::chrono::milliseconds(1000));
  });

  event_loop.Run();
}

// Test that if the binary changes out from under us that we note it in the
// FileState.
TEST_F(SubprocessTest, ChangeBinaryContents) {
  const std::string config_file =
      ::aos::testing::ArtifactPath("aos/events/pingpong_config.json");

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(config_file);
  aos::ShmEventLoop event_loop(&config.message());

  // Create a local copy of the sleep binary so that we can delete it.
  const std::filesystem::path full_executable_path =
      absl::StrCat(aos::testing::TestTmpDir(), "/", "sleep_binary");
  aos::util::WriteStringToFileOrDie(
      full_executable_path.native(),
      aos::util::ReadFileToStringOrDie(ResolvePath("sleep").native()), S_IRWXU);

  const std::filesystem::path executable_name =
      absl::StrCat(aos::testing::TestTmpDir(), "/", "sleep_symlink");
  // Create a symlink that points to the actual binary, and test that a
  // Creating a symlink in particular lets us ensure that our logic actually
  // pays attention to the target file that we are running rather than the
  // symlink itself (it also saves us having to cp a binary somewhere where we
  // can overwrite it).
  std::filesystem::create_symlink(full_executable_path.native(),
                                  executable_name);

  // Wait until we are running, go through and test that various variations in
  // file state result in the expected behavior, and then exit.
  Application sleep(
      "sleep", executable_name.native(), &event_loop,
      [&sleep, &event_loop, executable_name, full_executable_path]() {
        switch (sleep.status()) {
          case aos::starter::State::RUNNING:
            EXPECT_EQ(aos::starter::FileState::NO_CHANGE,
                      sleep.UpdateFileState());
            // Delete the symlink; this should have no effect, because the
            // Application class should be looking at the original path.
            std::filesystem::remove(executable_name);
            EXPECT_EQ(aos::starter::FileState::NO_CHANGE,
                      sleep.UpdateFileState());
            // Delete the executable; it should be changed.
            std::filesystem::remove(full_executable_path);
            EXPECT_EQ(aos::starter::FileState::CHANGED,
                      sleep.UpdateFileState());
            // Replace the executable itself; it should be changed.
            aos::util::WriteStringToFileOrDie(full_executable_path.native(),
                                              "abcdef");
            EXPECT_EQ(aos::starter::FileState::CHANGED,
                      sleep.UpdateFileState());
            // Terminate.
            event_loop.Exit();
            break;
          case aos::starter::State::WAITING:
          case aos::starter::State::STARTING:
          case aos::starter::State::STOPPING:
          case aos::starter::State::STOPPED:
            EXPECT_EQ(aos::starter::FileState::NOT_RUNNING,
                      sleep.UpdateFileState());
            break;
        }
      });
  ASSERT_FALSE(sleep.autorestart());
  // Ensure that the subprocess will run longer than we care about (we just call
  // Terminate() below to stop it).
  sleep.set_args({"1000"});

  sleep.Start();
  aos::TimerHandler *exit_timer = event_loop.AddTimer([&event_loop]() {
    event_loop.Exit();
    FAIL() << "We should have already exited.";
  });
  event_loop.OnRun([&event_loop, exit_timer]() {
    exit_timer->Schedule(event_loop.monotonic_now() +
                         std::chrono::milliseconds(5000));
  });

  event_loop.Run();
  sleep.Terminate();
}

class ResolvePathTest : public ::testing::Test {
 protected:
  ResolvePathTest() {
    // Before doing anything else,
    if (getenv("PATH") != nullptr) {
      original_path_ = getenv("PATH");
      PCHECK(0 == unsetenv("PATH"));
    }
  }

  ~ResolvePathTest() {
    if (!original_path_.empty()) {
      PCHECK(0 == setenv("PATH", original_path_.c_str(), /*overwrite=*/1));
    } else {
      PCHECK(0 == unsetenv("PATH"));
    }
  }

  std::filesystem::path GetLocalPath(const std::string filename) {
    return absl::StrCat(aos::testing::TestTmpDir(), "/", filename);
  }

  std::filesystem::path CreateFile(const std::string filename) {
    const std::filesystem::path file = GetLocalPath(filename);
    VLOG(2) << "Creating file at " << file;
    util::WriteStringToFileOrDie(file.native(), "contents");
    return file;
  }

  void SetPath(const std::vector<std::string> &path) {
    PCHECK(0 ==
           setenv("PATH", absl::StrJoin(path, ":").c_str(), /*overwrite=*/1));
  }

  // Keep track of original PATH environment variable so that we can restore
  // it.
  std::string original_path_;
};

// Tests that we can resolve paths when there is no PATH environment variable.
TEST_F(ResolvePathTest, ResolveWithUnsetPath) {
  const std::filesystem::path local_echo = CreateFile("echo");
  // Because the default path will be in /bin and /usr/bin (typically), we have
  // to choose some utility that we can reasonably expect to be available in the
  // test environment.
  const std::filesystem::path echo_path = ResolvePath("echo");
  EXPECT_THAT((std::vector<std::string>{"/bin/echo", "/usr/bin/echo"}),
              ::testing::Contains(echo_path.native()));

  // Test that a file with /'s in the name ignores the PATH.
  const std::filesystem::path local_echo_path =
      ResolvePath(local_echo.native());
  EXPECT_EQ(local_echo_path, local_echo);
}

// Test that when the PATH environment variable is set that we can use it.
TEST_F(ResolvePathTest, ResolveWithPath) {
  const std::filesystem::path local_folder = GetLocalPath("bin/");
  const std::filesystem::path local_folder2 = GetLocalPath("bin2/");
  aos::util::MkdirP(local_folder.native(), S_IRWXU);
  aos::util::MkdirP(local_folder2.native(), S_IRWXU);
  SetPath({local_folder, local_folder2});
  const std::filesystem::path binary = CreateFile("bin/binary");
  const std::filesystem::path duplicate_binary = CreateFile("bin2/binary");
  const std::filesystem::path other_name = CreateFile("bin2/other_name");

  EXPECT_EQ(binary, ResolvePath("binary"));
  EXPECT_EQ(other_name, ResolvePath("other_name"));
  // And check that if we specify the full path for the duplicate binary that we
  // can find it.
  EXPECT_EQ(duplicate_binary, ResolvePath(duplicate_binary.native()));
}

// Test that we fail to find non-existent files.
TEST_F(ResolvePathTest, DieOnFakeFile) {
  // Fail to find something that searches the PATH.
  SetPath({"foo", "bar"});
  EXPECT_DEATH(ResolvePath("fake_file"), "Unable to resolve");

  // Fail to find a local file.
  EXPECT_DEATH(ResolvePath("./fake_file"), "./fake_file does not exist");
}

}  // namespace aos::starter::testing
