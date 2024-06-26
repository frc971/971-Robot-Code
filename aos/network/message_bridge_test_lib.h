#ifndef AOS_NETWORK_MESSAGE_BRIDGE_TEST_LIB_H_
#define AOS_NETWORK_MESSAGE_BRIDGE_TEST_LIB_H_
#include <chrono>
#include <thread>

#include "absl/flags/reflection.h"
#include "absl/strings/str_cat.h"
#include "gtest/gtest.h"

#include "aos/events/ping_generated.h"
#include "aos/events/pong_generated.h"
#include "aos/ipc_lib/event.h"
#include "aos/network/message_bridge_client_lib.h"
#include "aos/network/message_bridge_protocol.h"
#include "aos/network/message_bridge_server_lib.h"
#include "aos/network/team_number.h"
#include "aos/sha256.h"
#include "aos/testing/path.h"
#include "aos/util/file.h"
namespace aos::message_bridge::testing {

namespace chrono = std::chrono;

// Class to manage starting and stopping a thread with an event loop in it.  The
// thread is guarenteed to be running before the constructor exits.
class ThreadedEventLoopRunner {
 public:
  ThreadedEventLoopRunner(aos::ShmEventLoop *event_loop);

  ~ThreadedEventLoopRunner();

  void Exit();

 private:
  aos::Event event_;
  aos::ShmEventLoop *event_loop_;
  std::thread my_thread_;
};

// Parameters to run all the tests with.
struct Param {
  // The config file to use.
  std::string config;
  // If true, the RemoteMessage channel should be shared between all the remote
  // channels.  If false, there will be 1 RemoteMessage channel per remote
  // channel.
  bool shared;
};

class PiNode {
 public:
  PiNode(const std::string node_name, const std::string host_name,
         const std::string app_name, const std::string config_filename);
  // OnPi* sets the global state necessary to pretend that a ShmEventLoop is on
  // the requisite system.
  void OnPi();
  void MakeServer(const std::string server_config_sha256 = "");
  void RunServer(const chrono::nanoseconds duration);
  void RunClient(const chrono::nanoseconds duration);
  void StartServer();
  void StopServer();
  void MakeClient();
  void StartClient();
  void StopClient();
  void MakeTest(const std::string test_app_name, const PiNode *other_node);
  void StartTest();
  void StopTest();

  const UUID boot_uuid_;
  const std::string node_name_;
  const std::string host_name_;
  const std::string app_name_;

  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  std::string config_sha256_;

  std::unique_ptr<aos::ShmEventLoop> server_event_loop_;
  std::unique_ptr<MessageBridgeServer> message_bridge_server_;
  std::unique_ptr<ThreadedEventLoopRunner> server_thread_;

  std::unique_ptr<aos::ShmEventLoop> client_event_loop_;
  std::unique_ptr<MessageBridgeClient> message_bridge_client_;
  std::unique_ptr<ThreadedEventLoopRunner> client_thread_;

  std::unique_ptr<aos::ShmEventLoop> test_event_loop_;
  std::unique_ptr<ThreadedEventLoopRunner> test_thread_;
};

class MessageBridgeParameterizedTest
    : public ::testing::TestWithParam<struct Param> {
 protected:
  MessageBridgeParameterizedTest();

  bool shared() const;

  absl::FlagSaver flag_saver_;

  PiNode pi1_;
  PiNode pi2_;

  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  std::string config_sha256_;
};

}  // namespace aos::message_bridge::testing

#endif  // AOS_NETWORK_MESSAGE_BRIDGE_TEST_LIB_H_
