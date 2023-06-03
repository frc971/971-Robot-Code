#include "aos/network/message_bridge_test_lib.h"

DECLARE_string(boot_uuid);

namespace aos {
void SetShmBase(const std::string_view base);

namespace message_bridge::testing {

namespace chrono = std::chrono;
using aos::testing::ArtifactPath;

std::string ShmBase(const std::string_view node) {
  const char *const tmpdir_c_str = getenv("TEST_TMPDIR");
  if (tmpdir_c_str != nullptr) {
    return absl::StrCat(tmpdir_c_str, "/", node);
  } else {
    return absl::StrCat("/dev/shm/", node);
  }
}

void DoSetShmBase(const std::string_view node) {
  aos::SetShmBase(ShmBase(node));
}

ThreadedEventLoopRunner::ThreadedEventLoopRunner(aos::ShmEventLoop *event_loop)
    : event_loop_(event_loop), my_thread_([this]() {
        LOG(INFO) << "Started " << event_loop_->name();
        event_loop_->OnRun([this]() { event_.Set(); });
        event_loop_->Run();
      }) {
  event_.Wait();
}

ThreadedEventLoopRunner::~ThreadedEventLoopRunner() { Exit(); }

void ThreadedEventLoopRunner::Exit() {
  if (my_thread_.joinable()) {
    event_loop_->Exit();
    my_thread_.join();
    my_thread_ = std::thread();
  }
}

MessageBridgeParameterizedTest::MessageBridgeParameterizedTest()
    : config(aos::configuration::ReadConfig(
          ArtifactPath(absl::StrCat("aos/network/", GetParam().config)))),
      config_sha256(Sha256(config.span())),
      pi1_boot_uuid_(UUID::Random()),
      pi2_boot_uuid_(UUID::Random()) {
  // Make sure that we clean up all the shared memory queues so that we cannot
  // inadvertently be influenced other tests or by previously run AOS
  // applications (in a fully sharded test running inside the bazel sandbox,
  // this should not matter).
  util::UnlinkRecursive(ShmBase("pi1"));
  util::UnlinkRecursive(ShmBase("pi2"));
}

bool MessageBridgeParameterizedTest::shared() const {
  return GetParam().shared;
}

void MessageBridgeParameterizedTest::OnPi1() {
  DoSetShmBase("pi1");
  FLAGS_override_hostname = "raspberrypi";
  FLAGS_boot_uuid = pi1_boot_uuid_.ToString();
}

void MessageBridgeParameterizedTest::OnPi2() {
  DoSetShmBase("pi2");
  FLAGS_override_hostname = "raspberrypi2";
  FLAGS_boot_uuid = pi2_boot_uuid_.ToString();
}

void MessageBridgeParameterizedTest::MakePi1Server(
    std::string server_config_sha256) {
  OnPi1();
  FLAGS_application_name = "pi1_message_bridge_server";
  pi1_server_event_loop =
      std::make_unique<aos::ShmEventLoop>(&config.message());
  pi1_server_event_loop->SetRuntimeRealtimePriority(1);
  pi1_message_bridge_server = std::make_unique<MessageBridgeServer>(
      pi1_server_event_loop.get(),
      server_config_sha256.size() == 0 ? config_sha256 : server_config_sha256);
}

void MessageBridgeParameterizedTest::RunPi1Server(
    chrono::nanoseconds duration) {
  // Set up a shutdown callback.
  aos::TimerHandler *const quit = pi1_server_event_loop->AddTimer(
      [this]() { pi1_server_event_loop->Exit(); });
  pi1_server_event_loop->OnRun([this, quit, duration]() {
    // Stop between timestamps, not exactly on them.
    quit->Schedule(pi1_server_event_loop->monotonic_now() + duration);
  });

  pi1_server_event_loop->Run();
}

void MessageBridgeParameterizedTest::StartPi1Server() {
  pi1_server_thread =
      std::make_unique<ThreadedEventLoopRunner>(pi1_server_event_loop.get());
}

void MessageBridgeParameterizedTest::StopPi1Server() {
  pi1_server_thread.reset();
  pi1_message_bridge_server.reset();
  pi1_server_event_loop.reset();
}

void MessageBridgeParameterizedTest::MakePi1Client() {
  OnPi1();
  FLAGS_application_name = "pi1_message_bridge_client";
  pi1_client_event_loop =
      std::make_unique<aos::ShmEventLoop>(&config.message());
  pi1_client_event_loop->SetRuntimeRealtimePriority(1);
  pi1_message_bridge_client = std::make_unique<MessageBridgeClient>(
      pi1_client_event_loop.get(), config_sha256);
}

void MessageBridgeParameterizedTest::StartPi1Client() {
  pi1_client_thread =
      std::make_unique<ThreadedEventLoopRunner>(pi1_client_event_loop.get());
}

void MessageBridgeParameterizedTest::StopPi1Client() {
  pi1_client_thread.reset();
  pi1_message_bridge_client.reset();
  pi1_client_event_loop.reset();
}

void MessageBridgeParameterizedTest::MakePi1Test() {
  OnPi1();
  FLAGS_application_name = "test1";
  pi1_test_event_loop = std::make_unique<aos::ShmEventLoop>(&config.message());

  pi1_test_event_loop->MakeWatcher(
      "/pi1/aos", [](const ServerStatistics &stats) {
        VLOG(1) << "/pi1/aos ServerStatistics " << FlatbufferToJson(&stats);
      });

  pi1_test_event_loop->MakeWatcher(
      "/pi1/aos", [](const ClientStatistics &stats) {
        VLOG(1) << "/pi1/aos ClientStatistics " << FlatbufferToJson(&stats);
      });

  pi1_test_event_loop->MakeWatcher("/pi1/aos", [](const Timestamp &timestamp) {
    VLOG(1) << "/pi1/aos Timestamp " << FlatbufferToJson(&timestamp);
  });
  pi1_test_event_loop->MakeWatcher("/pi2/aos", [this](
                                                   const Timestamp &timestamp) {
    VLOG(1) << "/pi2/aos Timestamp " << FlatbufferToJson(&timestamp);
    EXPECT_EQ(pi1_test_event_loop->context().source_boot_uuid, pi2_boot_uuid_);
  });
}

void MessageBridgeParameterizedTest::StartPi1Test() {
  pi1_test_thread =
      std::make_unique<ThreadedEventLoopRunner>(pi1_test_event_loop.get());
}

void MessageBridgeParameterizedTest::StopPi1Test() { pi1_test_thread.reset(); }

void MessageBridgeParameterizedTest::MakePi2Server() {
  OnPi2();
  FLAGS_application_name = "pi2_message_bridge_server";
  pi2_server_event_loop =
      std::make_unique<aos::ShmEventLoop>(&config.message());
  pi2_server_event_loop->SetRuntimeRealtimePriority(1);
  pi2_message_bridge_server = std::make_unique<MessageBridgeServer>(
      pi2_server_event_loop.get(), config_sha256);
}

void MessageBridgeParameterizedTest::RunPi2Server(
    chrono::nanoseconds duration) {
  // Schedule a shutdown callback.
  aos::TimerHandler *const quit = pi2_server_event_loop->AddTimer(
      [this]() { pi2_server_event_loop->Exit(); });
  pi2_server_event_loop->OnRun([this, quit, duration]() {
    // Stop between timestamps, not exactly on them.
    quit->Schedule(pi2_server_event_loop->monotonic_now() + duration);
  });

  pi2_server_event_loop->Run();
}

void MessageBridgeParameterizedTest::StartPi2Server() {
  pi2_server_thread =
      std::make_unique<ThreadedEventLoopRunner>(pi2_server_event_loop.get());
}

void MessageBridgeParameterizedTest::StopPi2Server() {
  pi2_server_thread.reset();
  pi2_message_bridge_server.reset();
  pi2_server_event_loop.reset();
}

void MessageBridgeParameterizedTest::MakePi2Client() {
  OnPi2();
  FLAGS_application_name = "pi2_message_bridge_client";
  pi2_client_event_loop =
      std::make_unique<aos::ShmEventLoop>(&config.message());
  pi2_client_event_loop->SetRuntimeRealtimePriority(1);
  pi2_message_bridge_client = std::make_unique<MessageBridgeClient>(
      pi2_client_event_loop.get(), config_sha256);
}

void MessageBridgeParameterizedTest::RunPi2Client(
    chrono::nanoseconds duration) {
  // Run for 5 seconds to make sure we have time to estimate the offset.
  aos::TimerHandler *const quit = pi2_client_event_loop->AddTimer(
      [this]() { pi2_client_event_loop->Exit(); });
  pi2_client_event_loop->OnRun([this, quit, duration]() {
    // Stop between timestamps, not exactly on them.
    quit->Schedule(pi2_client_event_loop->monotonic_now() + duration);
  });

  // And go!
  pi2_client_event_loop->Run();
}

void MessageBridgeParameterizedTest::StartPi2Client() {
  pi2_client_thread =
      std::make_unique<ThreadedEventLoopRunner>(pi2_client_event_loop.get());
}

void MessageBridgeParameterizedTest::StopPi2Client() {
  pi2_client_thread.reset();
  pi2_message_bridge_client.reset();
  pi2_client_event_loop.reset();
}

void MessageBridgeParameterizedTest::MakePi2Test() {
  OnPi2();
  FLAGS_application_name = "test2";
  pi2_test_event_loop = std::make_unique<aos::ShmEventLoop>(&config.message());

  pi2_test_event_loop->MakeWatcher(
      "/pi2/aos", [](const ServerStatistics &stats) {
        VLOG(1) << "/pi2/aos ServerStatistics " << FlatbufferToJson(&stats);
      });

  pi2_test_event_loop->MakeWatcher(
      "/pi2/aos", [](const ClientStatistics &stats) {
        VLOG(1) << "/pi2/aos ClientStatistics " << FlatbufferToJson(&stats);
      });

  pi2_test_event_loop->MakeWatcher("/pi1/aos", [this](
                                                   const Timestamp &timestamp) {
    VLOG(1) << "/pi1/aos Timestamp " << FlatbufferToJson(&timestamp);
    EXPECT_EQ(pi2_test_event_loop->context().source_boot_uuid, pi1_boot_uuid_);
  });
  pi2_test_event_loop->MakeWatcher("/pi2/aos", [](const Timestamp &timestamp) {
    VLOG(1) << "/pi2/aos Timestamp " << FlatbufferToJson(&timestamp);
  });
}

void MessageBridgeParameterizedTest::StartPi2Test() {
  pi2_test_thread =
      std::make_unique<ThreadedEventLoopRunner>(pi2_test_event_loop.get());
}

void MessageBridgeParameterizedTest::StopPi2Test() { pi2_test_thread.reset(); }
}  // namespace message_bridge::testing
}  // namespace aos
