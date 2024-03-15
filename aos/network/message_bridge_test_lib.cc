#include "aos/network/message_bridge_test_lib.h"

DECLARE_string(boot_uuid);

namespace aos::message_bridge::testing {

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
  aos::testing::SetShmBase(ShmBase(node));
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
    : pi1_("pi1", "raspberrypi", "pi1_message_bridge_server",
           GetParam().config),
      pi2_("pi2", "raspberrypi2", "pi2_message_bridge_client",
           GetParam().config),
      config_(aos::configuration::ReadConfig(GetParam().config)),
      config_sha256_(Sha256(config_.span())) {
  // Make sure that we clean up all the shared memory queues so that we cannot
  // inadvertently be influenced other tests or by previously run AOS
  // applications (in a fully sharded test running inside the bazel sandbox,
  // this should not matter).
  util::UnlinkRecursive(ShmBase(pi1_.node_name_));
  util::UnlinkRecursive(ShmBase(pi2_.node_name_));
}

bool MessageBridgeParameterizedTest::shared() const {
  return GetParam().shared;
}

PiNode::PiNode(const std::string node_name, const std::string host_name,
               const std::string app_name, const std::string config_filename)
    : boot_uuid_(UUID::Random()),
      node_name_(node_name),
      host_name_(host_name),
      app_name_(app_name),
      config_(aos::configuration::ReadConfig(config_filename)),
      config_sha256_(Sha256(config_.span())) {}

void PiNode::OnPi() {
  DoSetShmBase(node_name_);
  FLAGS_override_hostname = host_name_;
  FLAGS_boot_uuid = boot_uuid_.ToString();
}

void PiNode::MakeServer(const std::string server_config_sha256) {
  OnPi();
  LOG(INFO) << "Making " << node_name_ << " server";
  FLAGS_application_name = app_name_;
  server_event_loop_ = std::make_unique<aos::ShmEventLoop>(&config_.message());
  server_event_loop_->SetRuntimeRealtimePriority(1);
  message_bridge_server_ = std::make_unique<MessageBridgeServer>(
      server_event_loop_.get(),
      server_config_sha256.size() == 0 ? config_sha256_ : server_config_sha256,
      SctpAuthMethod::kNoAuth);
}

void PiNode::RunServer(const chrono::nanoseconds duration) {
  LOG(INFO) << "Running " << node_name_ << " server";
  // Set up a shutdown callback.
  aos::TimerHandler *const quit =
      server_event_loop_->AddTimer([this]() { server_event_loop_->Exit(); });
  server_event_loop_->OnRun([this, quit, duration]() {
    // Stop between timestamps, not exactly on them.
    quit->Schedule(server_event_loop_->monotonic_now() + duration);
  });

  server_event_loop_->Run();
}

void PiNode::StartServer() {
  LOG(INFO) << "Starting " << node_name_ << " server";
  server_thread_ =
      std::make_unique<ThreadedEventLoopRunner>(server_event_loop_.get());
}

void PiNode::StopServer() {
  LOG(INFO) << "Stopping " << node_name_ << " server";
  server_thread_.reset();
  message_bridge_server_.reset();
  server_event_loop_.reset();
}

void PiNode::MakeClient() {
  OnPi();
  LOG(INFO) << "Making " << node_name_ << " client";
  FLAGS_application_name = app_name_;
  client_event_loop_ = std::make_unique<aos::ShmEventLoop>(&config_.message());
  client_event_loop_->SetRuntimeRealtimePriority(1);
  message_bridge_client_ = std::make_unique<MessageBridgeClient>(
      client_event_loop_.get(), config_sha256_, SctpAuthMethod::kNoAuth);
}

void PiNode::StartClient() {
  LOG(INFO) << "Starting " << node_name_ << " client";
  client_thread_ =
      std::make_unique<ThreadedEventLoopRunner>(client_event_loop_.get());
}

void PiNode::StopClient() {
  LOG(INFO) << "Stopping " << node_name_ << " client";
  client_thread_.reset();
  message_bridge_client_.reset();
  client_event_loop_.reset();
}

void PiNode::MakeTest(const std::string test_app_name,
                      const PiNode *other_node) {
  OnPi();
  LOG(INFO) << "Making " << node_name_ << " test";
  FLAGS_application_name = test_app_name;
  test_event_loop_ = std::make_unique<aos::ShmEventLoop>(&config_.message());

  std::string channel_name = "/" + node_name_ + "/aos";
  test_event_loop_->MakeWatcher(
      channel_name, [channel_name](const ServerStatistics &stats) {
        VLOG(1) << channel_name << " ServerStatistics "
                << FlatbufferToJson(&stats);
      });

  test_event_loop_->MakeWatcher(
      channel_name, [channel_name](const ClientStatistics &stats) {
        VLOG(1) << channel_name << " ClientStatistics "
                << FlatbufferToJson(&stats);
      });

  test_event_loop_->MakeWatcher(channel_name,
                                [channel_name](const Timestamp &timestamp) {
                                  VLOG(1) << channel_name << " Timestamp "
                                          << FlatbufferToJson(&timestamp);
                                });
  std::string other_channel_name = "/" + other_node->node_name_ + "/aos";
  test_event_loop_->MakeWatcher(
      other_channel_name,
      [this, other_channel_name, other_node](const Timestamp &timestamp) {
        VLOG(1) << other_channel_name << " Timestamp "
                << FlatbufferToJson(&timestamp);
        EXPECT_EQ(test_event_loop_->context().source_boot_uuid,
                  other_node->boot_uuid_);
      });
}

void PiNode::StartTest() {
  LOG(INFO) << "Starting " << node_name_ << " test";
  test_thread_ =
      std::make_unique<ThreadedEventLoopRunner>(test_event_loop_.get());
}

void PiNode::StopTest() {
  LOG(INFO) << "Stopping " << node_name_ << " test";
  test_thread_.reset();
}

void PiNode::RunClient(const chrono::nanoseconds duration) {
  LOG(INFO) << "Running pi2 client";
  // Run for 5 seconds to make sure we have time to estimate the offset.
  aos::TimerHandler *const quit =
      client_event_loop_->AddTimer([this]() { client_event_loop_->Exit(); });
  client_event_loop_->OnRun([this, quit, duration]() {
    // Stop between timestamps, not exactly on them.
    quit->Schedule(client_event_loop_->monotonic_now() + duration);
  });

  // And go!
  client_event_loop_->Run();
}

}  // namespace aos::message_bridge::testing
