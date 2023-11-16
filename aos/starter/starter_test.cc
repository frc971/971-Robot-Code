#include <chrono>
#include <csignal>
#include <future>
#include <thread>

#include "gtest/gtest.h"

#include "aos/events/ping_generated.h"
#include "aos/events/pong_generated.h"
#include "aos/ipc_lib/event.h"
#include "aos/network/team_number.h"
#include "aos/starter/starter_rpc_lib.h"
#include "aos/starter/starterd_lib.h"
#include "aos/testing/path.h"
#include "aos/testing/tmpdir.h"
#include "aos/util/file.h"

using aos::testing::ArtifactPath;

namespace aos {
namespace starter {

class ThreadedStarterRunner {
 public:
  ThreadedStarterRunner(Starter *starter)
      : my_thread_([this, starter]() {
          starter->event_loop()->OnRun([this]() { event_.Set(); });
          starter->Run();
        }) {
    event_.Wait();
  }

  ~ThreadedStarterRunner() { my_thread_.join(); }

 private:
  aos::Event event_;
  std::thread my_thread_;
};

class StarterdTest : public ::testing::Test {
 public:
  StarterdTest() {
    // Nuke the shm dir:
    aos::util::UnlinkRecursive(FLAGS_shm_base);
  }

 protected:
  void SetupStarterCleanup(aos::starter::Starter *starter) {
    starter->event_loop()
        ->AddTimer([this, starter]() {
          if (test_done_) {
            starter->Cleanup();
          }
        })
        ->Schedule(starter->event_loop()->monotonic_now(),
                   std::chrono::milliseconds(100));
  }

  gflags::FlagSaver flag_saver_;
  // Used to track when the test completes so that we can clean up the starter
  // in its thread.
  std::atomic<bool> test_done_{false};
};

struct TestParams {
  std::string config;
  std::string hostname;
};

class StarterdConfigParamTest
    : public StarterdTest,
      public ::testing::WithParamInterface<TestParams> {};

TEST_P(StarterdConfigParamTest, MultiNodeStartStopTest) {
  gflags::FlagSaver flag_saver;
  FLAGS_override_hostname = GetParam().hostname;
  const std::string config_file = ArtifactPath(GetParam().config);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(config_file);

  auto new_config = aos::configuration::MergeWithConfig(
      &config.message(),
      absl::StrFormat(
          R"({"applications": [
                                  {
                                    "name": "ping",
                                    "executable_name": "%s",
                                    "nodes": ["pi1"],
                                    "args": ["--shm_base", "%s", "--config", "%s", "--override_hostname", "%s"]
                                  },
                                  {
                                    "name": "pong",
                                    "executable_name": "%s",
                                    "nodes": ["pi1"],
                                    "args": ["--shm_base", "%s", "--config", "%s", "--override_hostname", "%s"]
                                  }
                                ]})",
          ArtifactPath("aos/events/ping"), FLAGS_shm_base, config_file,
          GetParam().hostname, ArtifactPath("aos/events/pong"), FLAGS_shm_base,
          config_file, GetParam().hostname));

  const aos::Configuration *config_msg = &new_config.message();

  // Set up starter with config file
  aos::starter::Starter starter(config_msg);

  // Create an event loop to watch for ping messages, verifying it actually
  // started.
  aos::ShmEventLoop watcher_loop(config_msg);
  watcher_loop.SkipAosLog();

  aos::ShmEventLoop client_loop(config_msg);
  client_loop.SkipAosLog();
  StarterClient client(&client_loop);
  client.SetTimeoutHandler(
      []() { FAIL() << ": Command should not have timed out."; });
  bool success = false;
  client.SetSuccessHandler([&success, &client_loop]() {
    client_loop.Exit();
    success = true;
  });

  watcher_loop
      .AddTimer([&watcher_loop] {
        watcher_loop.Exit();
        FAIL();
      })
      ->Schedule(watcher_loop.monotonic_now() + std::chrono::seconds(7));

  std::atomic<int> test_stage = 0;
  // Watch on the client loop since we need to interact with the StarterClient.
  client_loop.MakeWatcher("/test", [&test_stage, &client,
                                    &client_loop](const aos::examples::Ping &) {
    switch (test_stage) {
      case 1: {
        test_stage = 2;
        break;
      }
      case 2: {
        {
          client.SendCommands({{Command::STOP, "ping", {client_loop.node()}}},
                              std::chrono::seconds(3));
        }
        test_stage = 3;
        break;
      }
    }
  });

  watcher_loop.MakeWatcher(
      "/aos", [&test_stage, &watcher_loop](const aos::starter::Status &status) {
        const aos::starter::ApplicationStatus *app_status =
            FindApplicationStatus(status, "ping");
        if (app_status == nullptr) {
          return;
        }

        switch (test_stage) {
          case 0: {
            if (app_status->has_state() &&
                app_status->state() == aos::starter::State::RUNNING) {
              test_stage = 1;
            }
            break;
          }

          case 3: {
            if (app_status->has_state() &&
                app_status->state() == aos::starter::State::STOPPED) {
              watcher_loop.Exit();
              SUCCEED();
            }
            break;
          }
        }
      });

  SetupStarterCleanup(&starter);

  ThreadedStarterRunner starterd_thread(&starter);

  aos::Event event;
  client_loop.OnRun([&event]() { event.Set(); });
  std::thread client_thread([&client_loop] { client_loop.Run(); });
  event.Wait();

  watcher_loop.Run();
  test_done_ = true;
  client_thread.join();
  ASSERT_TRUE(success);
}

INSTANTIATE_TEST_SUITE_P(
    StarterdConfigParamTest, StarterdConfigParamTest,
    ::testing::Values(TestParams{"aos/events/pingpong_config.json", ""},
                      TestParams{"aos/starter/multinode_pingpong_config.json",
                                 "pi1"}));

TEST_F(StarterdTest, DeathTest) {
  const std::string config_file =
      ArtifactPath("aos/events/pingpong_config.json");

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(config_file);

  auto new_config = aos::configuration::MergeWithConfig(
      &config.message(), absl::StrFormat(
                             R"({"applications": [
                                  {
                                    "name": "ping",
                                    "executable_name": "%s",
                                    "args": ["--shm_base", "%s"]
                                  },
                                  {
                                    "name": "pong",
                                    "executable_name": "%s",
                                    "args": ["--shm_base", "%s"]
                                  }
                                ]})",
                             ArtifactPath("aos/events/ping"), FLAGS_shm_base,
                             ArtifactPath("aos/events/pong"), FLAGS_shm_base));

  const aos::Configuration *config_msg = &new_config.message();

  // Set up starter with config file
  aos::starter::Starter starter(config_msg);

  // Create an event loop to watch for ping messages, verifying it actually
  // started.
  aos::ShmEventLoop watcher_loop(config_msg);
  watcher_loop.SkipAosLog();

  watcher_loop
      .AddTimer([&watcher_loop] {
        watcher_loop.Exit();
        FAIL();
      })
      ->Schedule(watcher_loop.monotonic_now() + std::chrono::seconds(11));

  int test_stage = 0;
  uint64_t id;

  watcher_loop.MakeWatcher("/aos", [&test_stage, &watcher_loop,
                                    &id](const aos::starter::Status &status) {
    const aos::starter::ApplicationStatus *app_status =
        FindApplicationStatus(status, "ping");
    if (app_status == nullptr) {
      return;
    }

    switch (test_stage) {
      case 0: {
        if (app_status->has_state() &&
            app_status->state() == aos::starter::State::RUNNING) {
          LOG(INFO) << "Ping is running";
          test_stage = 1;
          ASSERT_TRUE(app_status->has_pid());
          ASSERT_TRUE(kill(app_status->pid(), SIGINT) != -1);
          ASSERT_TRUE(app_status->has_id());
          id = app_status->id();
        }
        break;
      }

      case 1: {
        if (app_status->has_state() &&
            app_status->state() == aos::starter::State::RUNNING &&
            app_status->has_id() && app_status->id() != id) {
          LOG(INFO) << "Ping restarted";
          watcher_loop.Exit();
          SUCCEED();
        }
        break;
      }
    }
  });

  SetupStarterCleanup(&starter);

  ThreadedStarterRunner starterd_thread(&starter);
  watcher_loop.Run();

  test_done_ = true;
}

TEST_F(StarterdTest, Autostart) {
  const std::string config_file =
      ArtifactPath("aos/events/pingpong_config.json");

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(config_file);

  auto new_config = aos::configuration::MergeWithConfig(
      &config.message(), absl::StrFormat(
                             R"({"applications": [
                                  {
                                    "name": "ping",
                                    "executable_name": "%s",
                                    "args": ["--shm_base", "%s"],
                                    "autostart": false
                                  },
                                  {
                                    "name": "pong",
                                    "executable_name": "%s",
                                    "args": ["--shm_base", "%s"]
                                  }
                                ]})",
                             ArtifactPath("aos/events/ping"), FLAGS_shm_base,
                             ArtifactPath("aos/events/pong"), FLAGS_shm_base));

  const aos::Configuration *config_msg = &new_config.message();

  // Set up starter with config file
  aos::starter::Starter starter(config_msg);

  // Create an event loop to watch for the application starting up.
  aos::ShmEventLoop watcher_loop(config_msg);
  watcher_loop.SkipAosLog();

  watcher_loop
      .AddTimer([&watcher_loop] {
        watcher_loop.Exit();
        FAIL();
      })
      ->Schedule(watcher_loop.monotonic_now() + std::chrono::seconds(7));

  int pong_running_count = 0;
  watcher_loop.MakeWatcher("/aos", [&watcher_loop, &pong_running_count](
                                       const aos::starter::Status &status) {
    const aos::starter::ApplicationStatus *ping_app_status =
        FindApplicationStatus(status, "ping");
    const aos::starter::ApplicationStatus *pong_app_status =
        FindApplicationStatus(status, "pong");
    if (ping_app_status == nullptr || pong_app_status == nullptr) {
      return;
    }

    if (ping_app_status->has_state() &&
        ping_app_status->state() != aos::starter::State::STOPPED) {
      watcher_loop.Exit();
      FAIL();
    }
    if (pong_app_status->has_state() &&
        pong_app_status->state() == aos::starter::State::RUNNING) {
      ++pong_running_count;
      // Sometimes if the timing for everything is *just* off, then the
      // process_info will say that the process name is "starter_test" because
      // it grabbed the name after the fork() but before the execvp(). To
      // protect against that, wait an extra cycle. If things aren't fixed by
      // the second cycle, then that is a problem.
      if (pong_running_count < 3) {
        return;
      }
      ASSERT_TRUE(pong_app_status->has_process_info());
      ASSERT_EQ("pong", pong_app_status->process_info()->name()->string_view())
          << aos::FlatbufferToJson(&status);
      ASSERT_EQ(pong_app_status->pid(), pong_app_status->process_info()->pid());
      ASSERT_TRUE(pong_app_status->process_info()->has_cpu_usage());
      ASSERT_LE(0.0, pong_app_status->process_info()->cpu_usage());
      ASSERT_TRUE(pong_app_status->has_has_active_timing_report());
      ASSERT_TRUE(pong_app_status->has_active_timing_report());
      watcher_loop.Exit();
      SUCCEED();
    }
  });

  SetupStarterCleanup(&starter);

  ThreadedStarterRunner starterd_thread(&starter);
  watcher_loop.Run();

  test_done_ = true;
}

// Tests that starterd respects autorestart.
TEST_F(StarterdTest, DeathNoRestartTest) {
  const std::string config_file =
      ArtifactPath("aos/events/pingpong_config.json");

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(config_file);

  auto new_config = aos::configuration::MergeWithConfig(
      &config.message(), absl::StrFormat(
                             R"({"applications": [
                                  {
                                    "name": "ping",
                                    "executable_name": "%s",
                                    "args": ["--shm_base", "%s"],
                                    "autorestart": false
                                  },
                                  {
                                    "name": "pong",
                                    "executable_name": "%s",
                                    "args": ["--shm_base", "%s"]
                                  }
                                ]})",
                             ArtifactPath("aos/events/ping"), FLAGS_shm_base,
                             ArtifactPath("aos/events/pong"), FLAGS_shm_base));

  const aos::Configuration *config_msg = &new_config.message();

  // Set up starter with config file
  aos::starter::Starter starter(config_msg);

  // Create an event loop to watch for the Status message to watch the state
  // transitions.
  aos::ShmEventLoop watcher_loop(config_msg);
  watcher_loop.SkipAosLog();

  watcher_loop
      .AddTimer([&watcher_loop] {
        watcher_loop.Exit();
        SUCCEED();
      })
      ->Schedule(watcher_loop.monotonic_now() + std::chrono::seconds(11));

  int test_stage = 0;
  uint64_t id;

  watcher_loop.MakeWatcher("/aos", [&test_stage, &watcher_loop,
                                    &id](const aos::starter::Status &status) {
    const aos::starter::ApplicationStatus *app_status =
        FindApplicationStatus(status, "ping");
    if (app_status == nullptr) {
      return;
    }

    switch (test_stage) {
      case 0: {
        if (app_status->has_state() &&
            app_status->state() == aos::starter::State::RUNNING) {
          LOG(INFO) << "Ping is running";
          test_stage = 1;
          ASSERT_TRUE(app_status->has_pid());
          ASSERT_TRUE(kill(app_status->pid(), SIGINT) != -1);
          ASSERT_TRUE(app_status->has_id());
          id = app_status->id();
        }
        break;
      }

      case 1: {
        if (app_status->has_state() &&
            app_status->state() == aos::starter::State::RUNNING &&
            app_status->has_id() && app_status->id() != id) {
          LOG(INFO) << "Ping restarted, it shouldn't...";
          watcher_loop.Exit();
          FAIL();
        }
        break;
      }
    }
  });

  SetupStarterCleanup(&starter);

  ThreadedStarterRunner starterd_thread(&starter);
  watcher_loop.Run();

  test_done_ = true;
}

TEST_F(StarterdTest, StarterChainTest) {
  // This test was written in response to a bug that was found
  // in StarterClient::Succeed. The bug caused the timeout handler
  // to be reset after the success handler was called.
  // the bug has been fixed, and this test will ensure it does
  // not regress.
  const std::string config_file =
      ArtifactPath("aos/events/pingpong_config.json");
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(config_file);
  auto new_config = aos::configuration::MergeWithConfig(
      &config.message(), absl::StrFormat(
                             R"({"applications": [
                                {
                                  "name": "ping",
                                  "executable_name": "%s",
                                  "args": ["--shm_base", "%s"],
                                  "autorestart": false
                                },
                                {
                                  "name": "pong",
                                  "executable_name": "%s",
                                  "args": ["--shm_base", "%s"]
                                }
                              ]})",
                             ArtifactPath("aos/events/ping"), FLAGS_shm_base,
                             ArtifactPath("aos/events/pong"), FLAGS_shm_base));

  const aos::Configuration *config_msg = &new_config.message();
  // Set up starter with config file
  aos::starter::Starter starter(config_msg);
  aos::ShmEventLoop client_loop(config_msg);
  client_loop.SkipAosLog();
  StarterClient client(&client_loop);
  bool success = false;
  auto client_node = client_loop.node();

  // limit the amount of time we will wait for the test to finish.
  client_loop
      .AddTimer([&client_loop] {
        client_loop.Exit();
        FAIL() << "ERROR: The test has failed, the watcher has timed out. "
                  "The chain of stages defined below did not complete "
                  "within the time limit.";
      })
      ->Schedule(client_loop.monotonic_now() + std::chrono::seconds(20));

  // variables have been defined, here we define the body of the test.
  // We want stage1 to succeed, triggering stage2.
  // We want stage2 to timeout, triggering stage3.

  auto stage3 = [&client_loop, &success]() {
    LOG(INFO) << "Begin stage3.";
    SUCCEED();
    success = true;
    client_loop.Exit();
    LOG(INFO) << "End stage3.";
  };
  auto stage2 = [this, &starter, &client, &client_node, &stage3] {
    LOG(INFO) << "Begin stage2";
    test_done_ = true;  // trigger `starter` to exit.

    // wait for the starter event loop to close, so we can
    // intentionally trigger a timeout.
    int attempts = 0;
    while (starter.event_loop()->is_running()) {
      ++attempts;
      if (attempts > 5) {
        LOG(INFO) << "Timeout while waiting for starter to exit";
        return;
      }
      LOG(INFO) << "Waiting for starter to close.";
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    client.SetTimeoutHandler(stage3);
    client.SetSuccessHandler([]() {
      LOG(INFO) << "stage3 success handler called.";
      FAIL() << ": Command should not have succeeded here.";
    });
    // we want this command to timeout
    client.SendCommands({{Command::START, "ping", {client_node}}},
                        std::chrono::seconds(5));
    LOG(INFO) << "End stage2";
  };
  auto stage1 = [&client, &client_node, &stage2] {
    LOG(INFO) << "Begin stage1";
    client.SetTimeoutHandler(
        []() { FAIL() << ": Command should not have timed out."; });
    client.SetSuccessHandler(stage2);
    client.SendCommands({{Command::STOP, "ping", {client_node}}},
                        std::chrono::seconds(5));
    LOG(INFO) << "End stage1";
  };
  // start the test body
  client_loop.AddTimer(stage1)->Schedule(client_loop.monotonic_now() +
                                         std::chrono::milliseconds(1));

  // prepare the cleanup for starter. This will finish when we call
  // `test_done_ = true;`.
  SetupStarterCleanup(&starter);

  // run `starter.Run()` in a thread to simulate it running on
  // another process.
  ThreadedStarterRunner starterd_thread(&starter);

  client_loop.Run();
  EXPECT_TRUE(success);
  ASSERT_FALSE(starter.event_loop()->is_running());
}

}  // namespace starter
}  // namespace aos
