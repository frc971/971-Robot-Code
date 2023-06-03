#ifndef AOS_NETWORK_MESSAGE_BRIDGE_TEST_LIB_H_
#define AOS_NETWORK_MESSAGE_BRIDGE_TEST_LIB_H_
#include <chrono>
#include <thread>

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

class MessageBridgeParameterizedTest
    : public ::testing::TestWithParam<struct Param> {
 protected:
  MessageBridgeParameterizedTest();

  bool shared() const;

  // OnPi* sets the global state necessary to pretend that a ShmEventLoop is on
  // the requisite system.
  void OnPi1();

  void OnPi2();

  void MakePi1Server(std::string server_config_sha256 = "");

  void RunPi1Server(chrono::nanoseconds duration);

  void StartPi1Server();

  void StopPi1Server();

  void MakePi1Client();

  void StartPi1Client();

  void StopPi1Client();

  void MakePi1Test();

  void StartPi1Test();

  void StopPi1Test();

  void MakePi2Server();

  void RunPi2Server(chrono::nanoseconds duration);

  void StartPi2Server();

  void StopPi2Server();

  void MakePi2Client();

  void RunPi2Client(chrono::nanoseconds duration);

  void StartPi2Client();

  void StopPi2Client();

  void MakePi2Test();

  void StartPi2Test();

  void StopPi2Test();

  gflags::FlagSaver flag_saver_;

  aos::FlatbufferDetachedBuffer<aos::Configuration> config;
  std::string config_sha256;

  const UUID pi1_boot_uuid_;
  const UUID pi2_boot_uuid_;

  std::unique_ptr<aos::ShmEventLoop> pi1_server_event_loop;
  std::unique_ptr<MessageBridgeServer> pi1_message_bridge_server;
  std::unique_ptr<ThreadedEventLoopRunner> pi1_server_thread;

  std::unique_ptr<aos::ShmEventLoop> pi1_client_event_loop;
  std::unique_ptr<MessageBridgeClient> pi1_message_bridge_client;
  std::unique_ptr<ThreadedEventLoopRunner> pi1_client_thread;

  std::unique_ptr<aos::ShmEventLoop> pi1_test_event_loop;
  std::unique_ptr<ThreadedEventLoopRunner> pi1_test_thread;

  std::unique_ptr<aos::ShmEventLoop> pi2_server_event_loop;
  std::unique_ptr<MessageBridgeServer> pi2_message_bridge_server;
  std::unique_ptr<ThreadedEventLoopRunner> pi2_server_thread;

  std::unique_ptr<aos::ShmEventLoop> pi2_client_event_loop;
  std::unique_ptr<MessageBridgeClient> pi2_message_bridge_client;
  std::unique_ptr<ThreadedEventLoopRunner> pi2_client_thread;

  std::unique_ptr<aos::ShmEventLoop> pi2_test_event_loop;
  std::unique_ptr<ThreadedEventLoopRunner> pi2_test_thread;
};

}  // namespace aos::message_bridge::testing

#endif  // AOS_NETWORK_MESSAGE_BRIDGE_TEST_LIB_H_
