#include <signal.h>

#include <future>
#include <thread>

#include "aos/events/ping_generated.h"
#include "aos/events/pong_generated.h"
#include "aos/testing/tmpdir.h"
#include "gtest/gtest.h"
#include "starter_rpc_lib.h"
#include "starterd_lib.h"

TEST(StarterdTest, StartStopTest) {
  const std::string config_file = "aos/events/pingpong_config.json";

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(config_file);

  const std::string test_dir = aos::testing::TestTmpDir();

  auto new_config = aos::configuration::MergeWithConfig(
      &config.message(), absl::StrFormat(
                             R"({"applications": [
                                  {
                                    "name": "ping",
                                    "executable_name": "aos/events/ping",
                                    "args": ["--shm_base", "%s/aos"]
                                  },
                                  {
                                    "name": "pong",
                                    "executable_name": "aos/events/pong",
                                    "args": ["--shm_base", "%s/aos"]
                                  }
                                ]})",
                             test_dir, test_dir));

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
      ->Setup(watcher_loop.monotonic_now() + std::chrono::seconds(7));

  int test_stage = 0;
  watcher_loop.MakeWatcher(
      "/test", [&test_stage, config_msg](const aos::examples::Ping &) {
        switch (test_stage) {
          case 1: {
            test_stage = 2;
            break;
          }
          case 2: {
            std::thread([config_msg] {
              LOG(INFO) << "Send command";
              ASSERT_TRUE(aos::starter::SendCommandBlocking(
                  aos::starter::Command::STOP, "ping", config_msg,
                  std::chrono::seconds(3)));
            }).detach();
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

  std::thread starterd_thread([&starter] { starter.Run(); });
  watcher_loop.Run();

  starter.Cleanup();
  starterd_thread.join();
}

TEST(StarterdTest, DeathTest) {
  const std::string config_file = "aos/events/pingpong_config.json";

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(config_file);

  const std::string test_dir = aos::testing::TestTmpDir();

  auto new_config = aos::configuration::MergeWithConfig(
      &config.message(), absl::StrFormat(
                             R"({"applications": [
                                  {
                                    "name": "ping",
                                    "executable_name": "aos/events/ping",
                                    "args": ["--shm_base", "%s/aos"]
                                  },
                                  {
                                    "name": "pong",
                                    "executable_name": "aos/events/pong",
                                    "args": ["--shm_base", "%s/aos"]
                                  }
                                ]})",
                             test_dir, test_dir));

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
      ->Setup(watcher_loop.monotonic_now() + std::chrono::seconds(7));

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
          watcher_loop.Exit();
          SUCCEED();
        }
        break;
      }
    }
  });

  std::thread starterd_thread([&starter] { starter.Run(); });
  watcher_loop.Run();

  starter.Cleanup();
  starterd_thread.join();
}
