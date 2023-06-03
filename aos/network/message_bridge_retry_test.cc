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
#include "aos/network/message_bridge_test_lib.h"
#include "aos/network/team_number.h"
#include "aos/sha256.h"
#include "aos/testing/path.h"
#include "aos/util/file.h"

DECLARE_int32(force_wmem_max);

namespace aos {

namespace message_bridge {
namespace testing {

void SendPing(aos::Sender<examples::Ping> *sender, int value) {
  aos::Sender<examples::Ping>::Builder builder = sender->MakeBuilder();
  // Artificially inflate message size by adding a bunch of padding.
  builder.fbb()->CreateVector(std::vector<int>(1000, 0));
  examples::Ping::Builder ping_builder = builder.MakeBuilder<examples::Ping>();
  ping_builder.add_value(value);
  builder.CheckOk(builder.Send(ping_builder.Finish()));
}

// Test that if we fill up the kernel buffers then the message bridge code does
// indeed trigger (and succeed at triggering) its retry logic. Separated from
// the normal message bridge tests because triggering this originally seemed
// likely to be prone to extreme flakiness depending on the platform it is run
// on. In practice, it actually seems to be *more* reliable than the normal
// message_bridge_test, so we kept it separate.
TEST_P(MessageBridgeParameterizedTest, ReliableRetries) {
  // Set an absurdly small wmem max. This will help to trigger retries.
  FLAGS_force_wmem_max = 1024;
  OnPi1();

  FLAGS_application_name = "sender";
  aos::ShmEventLoop send_event_loop(&config.message());
  aos::Sender<examples::Ping> ping_sender =
      send_event_loop.MakeSender<examples::Ping>("/test");
  SendPing(&ping_sender, 1);
  aos::Fetcher<ServerStatistics> pi1_server_statistics_fetcher =
      send_event_loop.MakeFetcher<ServerStatistics>("/aos");

  MakePi1Server();
  MakePi1Client();

  // Now do it for "raspberrypi2", the client.
  OnPi2();

  MakePi2Server();

  aos::ShmEventLoop receive_event_loop(&config.message());
  aos::Fetcher<examples::Ping> ping_fetcher =
      receive_event_loop.MakeFetcher<examples::Ping>("/test");
  aos::Fetcher<ClientStatistics> pi2_client_statistics_fetcher =
      receive_event_loop.MakeFetcher<ClientStatistics>("/pi2/aos");

  // Before everything starts up, confirm there is no message.
  EXPECT_FALSE(ping_fetcher.Fetch());

  // Spin up the persistent pieces.
  StartPi1Server();
  StartPi1Client();
  StartPi2Server();

  {
    constexpr size_t kNumPingMessages = 25;
    // Now, spin up a client for 2 seconds.
    MakePi2Client();
    StartPi2Client();

    std::this_thread::sleep_for(std::chrono::seconds(2));

    for (size_t i = 0; i < kNumPingMessages; ++i) {
      SendPing(&ping_sender, i);
    }

    // Give plenty of time for retries to succeed.
    std::this_thread::sleep_for(std::chrono::seconds(5));

    StopPi2Client();

    // Confirm there is no detected duplicate packet.
    EXPECT_TRUE(pi2_client_statistics_fetcher.Fetch());
    EXPECT_GT(pi2_client_statistics_fetcher->connections()
                  ->Get(0)
                  ->received_packets(),
              kNumPingMessages);
    EXPECT_EQ(pi2_client_statistics_fetcher->connections()
                  ->Get(0)
                  ->duplicate_packets(),
              0u);

    EXPECT_EQ(pi2_client_statistics_fetcher->connections()
                  ->Get(0)
                  ->partial_deliveries(),
              0u);

    // Check that we received the reliable message that was sent before
    // starting.
    EXPECT_TRUE(ping_fetcher.FetchNext());
    EXPECT_EQ(ping_fetcher->value(), 1);

    // Check that we got all the messages sent while running.
    for (size_t i = 0; i < kNumPingMessages; ++i) {
      EXPECT_TRUE(ping_fetcher.FetchNext());
      EXPECT_EQ(ping_fetcher->value(), i);
    }

    EXPECT_TRUE(pi1_server_statistics_fetcher.Fetch());
    EXPECT_GT(
        pi1_server_statistics_fetcher->connections()->Get(0)->sent_packets(),
        kNumPingMessages);
    EXPECT_GT(
        pi1_server_statistics_fetcher->connections()->Get(0)->retry_count(), 0u)
        << FlatbufferToJson(pi1_server_statistics_fetcher.get());
  }

  // Shut everyone else down.
  StopPi1Client();
  StopPi2Server();
  StopPi1Server();
}

INSTANTIATE_TEST_SUITE_P(MessageBridgeTests, MessageBridgeParameterizedTest,
                         ::testing::Values(Param{
                             "message_bridge_test_common_config.json", false}));

}  // namespace testing
}  // namespace message_bridge
}  // namespace aos
