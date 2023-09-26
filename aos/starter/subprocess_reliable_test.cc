#include <signal.h>
#include <sys/types.h>

#include <filesystem>

#include "gtest/gtest.h"

#include "aos/events/shm_event_loop.h"
#include "aos/starter/subprocess.h"
#include "aos/testing/path.h"
#include "aos/testing/tmpdir.h"
#include "aos/util/file.h"

namespace aos::starter::testing {

namespace {
void Wait(pid_t pid) {
  int status;
  if (waitpid(pid, &status, 0) != pid) {
    if (errno != ECHILD) {
      PLOG(ERROR) << "Failed to wait for PID " << pid << ": " << status;
      FAIL();
    }
  }
  LOG(INFO) << "Succesfully waited for PID " << pid;
}

}  // namespace

// Validates that killing a child process right after startup doesn't have any
// unexpected consequences. The child process should exit even if it hasn't
// `exec()`d yet.
TEST(SubprocessTest, KillDuringStartup) {
  const std::string config_file =
      ::aos::testing::ArtifactPath("aos/events/pingpong_config.json");
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(config_file);
  aos::ShmEventLoop event_loop(&config.message());

  // Run an application that takes a really long time to exit. The exact details
  // here don't matter. We just need to to survive long enough until we can call
  // Terminate() below.
  auto application =
      std::make_unique<Application>("sleep", "sleep", &event_loop, []() {});
  application->set_args({"100"});

  // Track whether we exit via our own timer callback. We don't want to exit
  // because of any strange interactions with the child process.
  bool exited_as_expected = false;

  // Here's the sequence of events that we expect to see:
  // 1. Start child process.
  // 2. Stop child process (via `Terminate()`).
  // 3. Wait 1 second.
  // 4. Set `exited_as_expected` to `true`.
  // 5. Exit the event loop.
  //
  // At the end, if `exited_as_expected` is `false`, then something unexpected
  // happened and we failed the test here.
  aos::TimerHandler *shutdown_timer = event_loop.AddTimer([&]() {
    exited_as_expected = true;
    event_loop.Exit();
  });
  aos::TimerHandler *trigger_timer = event_loop.AddTimer([&]() {
    application->Start();
    application->Terminate();
    shutdown_timer->Schedule(event_loop.monotonic_now() +
                             std::chrono::seconds(1));
  });
  trigger_timer->Schedule(event_loop.monotonic_now());
  event_loop.Run();
  application->Stop();
  Wait(application->get_pid());

  EXPECT_TRUE(exited_as_expected) << "It looks like we killed ourselves.";
}

// Validates that the code in subprocess.cc doesn't accidentally block signals
// in the child process.
TEST(SubprocessTest, CanKillAfterStartup) {
  const std::string config_file =
      ::aos::testing::ArtifactPath("aos/events/pingpong_config.json");
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(config_file);
  aos::ShmEventLoop event_loop(&config.message());

  // We create a directory in which we create some files so this test here and
  // the subsequently created child process can "signal" one another. We roughly
  // expect the following sequence of events:
  // 1. Start the child process.
  // 2. Test waits for "startup" file to be created by child.
  // 3. Child process creates the "startup" file.
  // 4. Test sees "startup" file being created, sends SIGTERM to child.
  // 5. Child sees SIGTERM, creates "shutdown" file, and exits.
  // 6. Test waits for child to exit.
  // 7. Test validates that the "shutdown" file was created by the child.
  auto signal_dir = std::filesystem::path(aos::testing::TestTmpDir()) /
                    "startup_file_signals";
  ASSERT_TRUE(std::filesystem::create_directory(signal_dir));
  auto startup_signal_file = signal_dir / "startup";
  auto shutdown_signal_file = signal_dir / "shutdown";

  auto application = std::make_unique<Application>("/bin/bash", "/bin/bash",
                                                   &event_loop, []() {});
  application->set_args(
      {"-c", absl::StrCat("cleanup() { touch ", shutdown_signal_file.string(),
                          "; exit 0; }; trap cleanup SIGTERM; touch ",
                          startup_signal_file.string(),
                          "; while true; do sleep 0.1; done;")});

  // Wait for the child process to create the "startup" file.
  ASSERT_FALSE(std::filesystem::exists(startup_signal_file));
  application->Start();
  while (!std::filesystem::exists(startup_signal_file)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  ASSERT_FALSE(std::filesystem::exists(shutdown_signal_file));

  // Manually kill the application here. The Stop() and Terminate() helpers
  // trigger some timeout behaviour that interferes with the test here. This
  // should cause the child to exit and create the "shutdown" file.
  PCHECK(kill(application->get_pid(), SIGTERM) == 0);
  Wait(application->get_pid());
  ASSERT_TRUE(std::filesystem::exists(shutdown_signal_file));
}

}  // namespace aos::starter::testing
