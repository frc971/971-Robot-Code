#include <chrono>
#include <memory>
#include <thread>

#include "gmock/gmock-matchers.h"
#include "gtest/gtest.h"

#include "aos/configuration.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/message_bridge_auth_client_lib.h"
#include "aos/network/message_bridge_auth_server_lib.h"
#include "aos/network/sctp_config_generated.h"
#include "aos/network/sctp_config_request_generated.h"
#include "aos/testing/path.h"
#include "grpc/grpc.h"
#include "grpc/grpc_security.h"
#include "grpcpp/channel.h"
#include "grpcpp/create_channel.h"
#include "grpcpp/server_builder.h"

namespace aos::message_bridge::auth::testing {
namespace {

using ::aos::testing::ArtifactPath;
using ::grpc::Channel;
using ::grpc::Server;
using ::grpc::ServerBuilder;
using ::testing::ElementsAreArray;
using ::testing::Not;

using namespace ::std::chrono_literals;

void RunServer() {
  std::string server_address("127.0.0.1:1234");
  SctpConfigServer service;
  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  std::unique_ptr<Server> auth_server(builder.BuildAndStart());
  auth_server->Wait();
}

std::shared_ptr<Channel> MakeAuthClientChannel() {
  std::string server_address = "127.0.0.1:1234";
  std::shared_ptr<Channel> channel(
      grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials()));
  // Give 10s deadline to connect to server or bail.
  CHECK(channel->WaitForConnected(std::chrono::system_clock::now() + 10s))
      << "Couldn't connect to auth server.";
  return channel;
}

class MessageBridgeAuthTest : public ::testing::Test {
 public:
  MessageBridgeAuthTest()
      : config_(aos::configuration::ReadConfig(
            ArtifactPath("aos/network/message_bridge_auth_test_config.json"))),
        event_loop_factory_(&config_.message()),
        client_event_loop_(event_loop_factory_.MakeEventLoop("client")),
        auth_client_(client_event_loop_.get(), MakeAuthClientChannel()),
        mock_event_loop_(event_loop_factory_.MakeEventLoop("mock")),
        request_key_sender_(
            mock_event_loop_->MakeSender<SctpConfigRequest>("/aos")) {}

  static void SetUpTestSuite() { std::thread(RunServer).detach(); }

 private:
  FlatbufferDetachedBuffer<Configuration> config_;
  SimulatedEventLoopFactory event_loop_factory_;
  std::unique_ptr<EventLoop> client_event_loop_;
  MessageBridgeAuthClient auth_client_;

 protected:
  void RequestAuthKey() {
    LOG(INFO) << "Requesting auth key";
    auto sender = request_key_sender_.MakeBuilder();
    auto builder = sender.MakeBuilder<SctpConfigRequest>();
    builder.add_request_key(true);
    sender.CheckOk(sender.Send(builder.Finish()));
  }

  void RunFor(distributed_clock::duration duration) {
    event_loop_factory_.RunFor(duration);
  }

  std::unique_ptr<EventLoop> mock_event_loop_;
  Sender<SctpConfigRequest> request_key_sender_;
};

// The "obvious" test for the message bridge authentication server/client.
//
// It spins up the server (done once for the entire test suite), creates a
// client, and checks that we propagate the key from the server into the
// appropriate AOS channel upon an a key request.
TEST_F(MessageBridgeAuthTest, SmokeTest) {
  // How many times we receive a new key.
  int key_count = 0;
  std::vector<uint8_t> auth_key;
  mock_event_loop_->MakeWatcher("/aos", [&](const SctpConfig &config) {
    if (config.has_key()) {
      key_count++;
      if (auth_key.empty()) {
        auth_key.assign(config.key()->begin(), config.key()->end());
      }
      LOG(INFO) << "Got new auth key";
      // Key shouldn't change as we are running on the same server.
      EXPECT_THAT(auth_key,
                  ElementsAreArray(config.key()->begin(), config.key()->end()));
    }
  });

  RunFor(1000ms);
  RequestAuthKey();
  RunFor(1000ms);
  RequestAuthKey();
  RunFor(1000ms);
  EXPECT_FALSE(auth_key.empty());
  // We requested the key twice, we should get it back twice.
  EXPECT_EQ(key_count, 2);
}

}  // namespace
}  // namespace aos::message_bridge::auth::testing
