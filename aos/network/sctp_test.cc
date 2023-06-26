#include <unistd.h>

#include <chrono>
#include <functional>

#include "gflags/gflags.h"
#include "gmock/gmock-matchers.h"
#include "gtest/gtest.h"

#include "aos/events/epoll.h"
#include "aos/network/sctp_client.h"
#include "aos/network/sctp_lib.h"
#include "aos/network/sctp_server.h"

DECLARE_bool(disable_ipv6);

namespace aos::message_bridge::testing {

using ::aos::internal::EPoll;
using ::aos::internal::TimerFd;
using ::testing::ElementsAre;

using namespace ::std::chrono_literals;

constexpr int kPort = 19423;
constexpr int kStreams = 1;

namespace {
void EnableSctpAuthIfAvailable() {
#if HAS_SCTP_AUTH
  CHECK(system("/usr/sbin/sysctl net.sctp.auth_enable=1 || /sbin/sysctl "
               "net.sctp.auth_enable=1") == 0)
      << "Couldn't enable sctp authentication.";
#endif
}
}  // namespace

// An asynchronous SCTP handler. It takes an SCTP receiver (a.k.a SctpServer or
// SctpClient), and an `sctp_notification` handler and a `message` handler. It
// asynchronously routes incoming messages to the appropriate handler.
template <typename T>
class SctpReceiver {
 public:
  SctpReceiver(
      EPoll &epoll, T &receiver,
      std::function<void(T &, const union sctp_notification *)> on_notify,
      std::function<void(T &, std::vector<uint8_t>)> on_message)
      : epoll_(epoll),
        receiver_(receiver),
        on_notify_(std::move(on_notify)),
        on_message_(std::move(on_message)) {
    epoll_.OnReadable(receiver_.fd(), [this]() { Read(); });
  }

  ~SctpReceiver() { epoll_.DeleteFd(receiver_.fd()); }

 private:
  // Handles an incoming message by routing it to the apropriate handler.
  void Read() {
    aos::unique_c_ptr<Message> message = receiver_.Read();
    if (!message) {
      return;
    }

    switch (message->message_type) {
      case Message::kNotification: {
        const union sctp_notification *notification =
            reinterpret_cast<const union sctp_notification *>(message->data());
        on_notify_(receiver_, notification);
        break;
      }
      case Message::kMessage:
        on_message_(receiver_, std::vector(message->data(),
                                           message->data() + message->size));
        break;
      case Message::kOverflow:
        LOG(FATAL) << "Overflow";
    }
    receiver_.FreeMessage(std::move(message));
  }

  EPoll &epoll_;
  T &receiver_;
  std::function<void(T &, const union sctp_notification *)> on_notify_;
  std::function<void(T &, std::vector<uint8_t>)> on_message_;
};

// Base SctpTest class.
//
// The class provides a few virtual methods that should be overriden to define
// the behavior of the test.
class SctpTest : public ::testing::Test {
 public:
  SctpTest(std::vector<uint8_t> server_key = {},
           std::vector<uint8_t> client_key = {},
           SctpAuthMethod requested_authentication = SctpAuthMethod::kNoAuth,
           std::chrono::milliseconds timeout = 1000ms)
      : server_(kStreams, "", kPort, requested_authentication),
        client_("localhost", kPort, kStreams, "", 0, requested_authentication),
        client_receiver_(
            epoll_, client_,
            [this](SctpClient &client,
                   const union sctp_notification *notification) {
              HandleNotification(client, notification);
            },
            [this](SctpClient &client, std::vector<uint8_t> message) {
              HandleMessage(client, std::move(message));
            }),
        server_receiver_(
            epoll_, server_,
            [this](SctpServer &server,
                   const union sctp_notification *notification) {
              HandleNotification(server, notification);
            },
            [this](SctpServer &server, std::vector<uint8_t> message) {
              HandleMessage(server, std::move(message));
            }) {
    server_.SetAuthKey(server_key);
    client_.SetAuthKey(client_key);
    timeout_.SetTime(aos::monotonic_clock::now() + timeout,
                     std::chrono::milliseconds::zero());
    epoll_.OnReadable(timeout_.fd(), [this]() { TimeOut(); });
  }

  static void SetUpTestSuite() {
    EnableSctpAuthIfAvailable();
    // Buildkite seems to have issues with ipv6 sctp sockets...
    FLAGS_disable_ipv6 = true;
  }

  void SetUp() override { Run(); }

 protected:
  // Handles a server notification message.
  //
  // The default behaviour is to track the sctp association ID.
  virtual void HandleNotification(SctpServer &,
                                  const union sctp_notification *notification) {
    if (notification->sn_header.sn_type == SCTP_ASSOC_CHANGE) {
      assoc_ = notification->sn_assoc_change.sac_assoc_id;
    }
  }

  // Handles the client notification message.
  virtual void HandleNotification(SctpClient &,
                                  const union sctp_notification *) {}

  // Handles a server "data" message.
  virtual void HandleMessage(SctpServer &, std::vector<uint8_t>) {}
  // Handles a client "data" message.
  virtual void HandleMessage(SctpClient &, std::vector<uint8_t>) {}

  // Defines the "timeout" behaviour (fail by default).
  virtual void TimeOut() {
    Quit();
    FAIL() << "Timer expired";
  }

  virtual ~SctpTest() {}

  // Quit the test.
  void Quit() {
    epoll_.DeleteFd(timeout_.fd());
    epoll_.Quit();
  }
  void Run() { epoll_.Run(); }

  SctpServer server_;
  SctpClient client_;
  sctp_assoc_t assoc_ = 0;

 private:
  TimerFd timeout_;
  EPoll epoll_;
  SctpReceiver<SctpClient> client_receiver_;
  SctpReceiver<SctpServer> server_receiver_;
};

// Verifies we can ping the server, and the server replies.
class SctpPingPongTest : public SctpTest {
 public:
  SctpPingPongTest()
      : SctpTest({}, {}, SctpAuthMethod::kNoAuth, /*timeout=*/2s) {
    // Start by having the client send "ping".
    client_.Send(0, "ping", 0);
  }

  void HandleMessage(SctpServer &server,
                     std::vector<uint8_t> message) override {
    // Server should receive a ping message.
    EXPECT_THAT(message, ElementsAre('p', 'i', 'n', 'g'));
    got_ping_ = true;
    ASSERT_NE(assoc_, 0);
    // Reply with "pong".
    server.Send("pong", assoc_, 0, 0);
  }

  void HandleMessage(SctpClient &, std::vector<uint8_t> message) override {
    // Client should receive a "pong" message.
    EXPECT_THAT(message, ElementsAre('p', 'o', 'n', 'g'));
    got_pong_ = true;
    // We are done.
    Quit();
  }
  ~SctpPingPongTest() {
    // Check that we got the ping/pong messages.
    // This isn't strictly necessary as otherwise we would time out and fail
    // anyway.
    EXPECT_TRUE(got_ping_);
    EXPECT_TRUE(got_pong_);
  }

 protected:
  bool got_ping_ = false;
  bool got_pong_ = false;
};

TEST_F(SctpPingPongTest, Test) {}

#if HAS_SCTP_AUTH

// Same as SctpPingPongTest but with authentication keys. Both keys are the
// same so it should work the same way.
class SctpAuthTest : public SctpTest {
 public:
  SctpAuthTest()
      : SctpTest({1, 2, 3, 4, 5, 6}, {1, 2, 3, 4, 5, 6}, SctpAuthMethod::kAuth,
                 /*timeout*/ 20s) {
    // Start by having the client send "ping".
    client_.Send(0, "ping", 0);
  }

  void HandleMessage(SctpServer &server,
                     std::vector<uint8_t> message) override {
    // Server should receive a ping message.
    EXPECT_THAT(message, ElementsAre('p', 'i', 'n', 'g'));
    got_ping_ = true;
    ASSERT_NE(assoc_, 0);
    // Reply with "pong".
    server.Send("pong", assoc_, 0, 0);
  }
  void HandleMessage(SctpClient &, std::vector<uint8_t> message) override {
    // Client should receive a "pong" message.
    EXPECT_THAT(message, ElementsAre('p', 'o', 'n', 'g'));
    got_pong_ = true;
    // We are done.
    Quit();
  }
  ~SctpAuthTest() {
    EXPECT_TRUE(got_ping_);
    EXPECT_TRUE(got_pong_);
  }

 protected:
  bool got_ping_ = false;
  bool got_pong_ = false;
};

TEST_F(SctpAuthTest, Test) {}

// Tests that we can dynamically change the SCTP authentication key used.
class SctpChangingAuthKeysTest : public SctpTest {
 public:
  SctpChangingAuthKeysTest()
      : SctpTest({1, 2, 3, 4, 5, 6}, {1, 2, 3, 4, 5, 6},
                 SctpAuthMethod::kAuth) {
    // Start by having the client send "ping".
    client_.SetAuthKey({5, 4, 3, 2, 1});
    server_.SetAuthKey({5, 4, 3, 2, 1});
    client_.Send(0, "ping", 0);
  }

  void HandleMessage(SctpServer &server,
                     std::vector<uint8_t> message) override {
    // Server should receive a ping message.
    EXPECT_THAT(message, ElementsAre('p', 'i', 'n', 'g'));
    got_ping_ = true;
    ASSERT_NE(assoc_, 0);
    // Reply with "pong".
    server.Send("pong", assoc_, 0, 0);
  }
  void HandleMessage(SctpClient &, std::vector<uint8_t> message) override {
    // Client should receive a "pong" message.
    EXPECT_THAT(message, ElementsAre('p', 'o', 'n', 'g'));
    got_pong_ = true;
    // We are done.
    Quit();
  }

  ~SctpChangingAuthKeysTest() {
    EXPECT_TRUE(got_ping_);
    EXPECT_TRUE(got_pong_);
  }

 protected:
  bool got_ping_ = false;
  bool got_pong_ = false;
};

TEST_F(SctpChangingAuthKeysTest, Test) {}

// Keys don't match, we should send the `ping` message but the server should
// never receive it. We then time out as nothing calls Quit.
class SctpMismatchedAuthTest : public SctpTest {
 public:
  SctpMismatchedAuthTest()
      : SctpTest({1, 2, 3, 4, 5, 6}, {5, 6, 7, 8, 9, 10},
                 SctpAuthMethod::kAuth) {
    // Start by having the client send "ping".
    client_.Send(0, "ping", 0);
  }

  void HandleMessage(SctpServer &, std::vector<uint8_t>) override {
    FAIL() << "Authentication keys don't match. Message should be discarded";
    Quit();
  }

  // We expect to time out since we never get the message.
  void TimeOut() override { Quit(); }
};

TEST_F(SctpMismatchedAuthTest, Test) {}

// Same as SctpMismatchedAuthTest but the client uses the null key. We should
// see the same behaviour.
class SctpOneNullKeyTest : public SctpTest {
 public:
  SctpOneNullKeyTest()
      : SctpTest({1, 2, 3, 4, 5, 6}, {}, SctpAuthMethod::kAuth) {
    // Start by having the client send "ping".
    client_.Send(0, "ping", 0);
  }

  void HandleMessage(SctpServer &, std::vector<uint8_t>) override {
    FAIL() << "Authentication keys don't match. Message should be discarded";
    Quit();
  }

  // We expect to time out since we never get the message.
  void TimeOut() override { Quit(); }
};

TEST_F(SctpOneNullKeyTest, Test) {}

// If we want SCTP authentication but we don't set the auth keys, we shouldn't
// be able to send data.
class SctpAuthKeysNotSet : public SctpTest {
 public:
  SctpAuthKeysNotSet() : SctpTest({}, {}, SctpAuthMethod::kAuth) {
    // Start by having the client send "ping".
    client_.Send(0, "ping", 0);
  }

  void HandleMessage(SctpServer &, std::vector<uint8_t>) override {
    FAIL() << "Haven't setup authentication keys. Should not get message.";
    Quit();
  }

  // We expect to time out since we never get the message.
  void TimeOut() override { Quit(); }
};

TEST_F(SctpAuthKeysNotSet, Test) {}

#endif  // HAS_SCTP_AUTH

}  // namespace aos::message_bridge::testing
