#include <csignal>
#include <experimental/filesystem>
#include <future>
#include <thread>

#include "aos/events/ping_generated.h"
#include "aos/events/pong_generated.h"
#include "aos/network/team_number.h"
#include "aos/testing/path.h"
#include "aos/testing/tmpdir.h"
#include "gtest/gtest.h"
#include "starter_rpc_lib.h"
#include "starterd_lib.h"

using aos::testing::ArtifactPath;

namespace aos {
namespace starter {

class StarterdTest : public ::testing::Test {
 public:
  StarterdTest() : shm_dir_(aos::testing::TestTmpDir() + "/aos") {
    FLAGS_shm_base = shm_dir_;

    // Nuke the shm dir:
    std::experimental::filesystem::remove_all(shm_dir_);
  }

 protected:
  gflags::FlagSaver flag_saver_;
  std::string shm_dir_;
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
          ArtifactPath("aos/events/ping"), shm_dir_, config_file,
          GetParam().hostname, ArtifactPath("aos/events/pong"), shm_dir_,
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
      ->Setup(watcher_loop.monotonic_now() + std::chrono::seconds(7));

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

  std::thread starterd_thread([&starter] { starter.Run(); });
  std::thread client_thread([&client_loop] { client_loop.Run(); });
  watcher_loop.Run();

  starter.Cleanup();
  client_thread.join();
  starterd_thread.join();
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
                             ArtifactPath("aos/events/ping"), shm_dir_,
                             ArtifactPath("aos/events/pong"), shm_dir_));

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
      ->Setup(watcher_loop.monotonic_now() + std::chrono::seconds(11));

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

  std::thread starterd_thread([&starter] { starter.Run(); });
  watcher_loop.Run();

  starter.Cleanup();
  starterd_thread.join();
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
                             ArtifactPath("aos/events/ping"), shm_dir_,
                             ArtifactPath("aos/events/pong"), shm_dir_));

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
      ->Setup(watcher_loop.monotonic_now() + std::chrono::seconds(7));

  watcher_loop.MakeWatcher(
      "/aos", [&watcher_loop](const aos::starter::Status &status) {
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
          watcher_loop.Exit();
          SUCCEED();
        }
      });

  std::thread starterd_thread([&starter] { starter.Run(); });
  watcher_loop.Run();

  starter.Cleanup();
  starterd_thread.join();
}

}  // namespace starter
}  // namespace aos
