#include "gtest/gtest.h"

#include <chrono>
#include <thread>

#include "aos/events/ping_generated.h"
#include "aos/events/pong_generated.h"
#include "aos/network/message_bridge_client_lib.h"
#include "aos/network/message_bridge_server_lib.h"

DECLARE_string(override_hostname);
DECLARE_string(application_name);

namespace aos {
namespace message_bridge {
namespace testing {

namespace chrono = std::chrono;

// Test that we can send a ping message over sctp and receive it.
TEST(MessageBridgeTest, PingPong) {
  // This is rather annoying to set up.  We need to start up a client and
  // server, on the same node, but get them to think that they are on different
  // nodes.
  //
  // We then get to wait until they are connected.
  //
  // After they are connected, we send a Ping message.
  //
  // On the other end, we receive a Pong message.
  //
  // But, we need the client to not post directly to "/test" like it would in a
  // real system, otherwise we will re-send the ping message... So, use an
  // application specific map to have the client post somewhere else.
  //
  // To top this all off, each of these needs to be done with a ShmEventLoop,
  // which needs to run in a separate thread...  And it is really hard to get
  // everything started up reliably.  So just be super generous on timeouts and
  // hope for the best.  We can be more generous in the future if we need to.
  //
  // We are faking the application names by passing in --application_name=foo
  aos::FlatbufferDetachedBuffer<aos::Configuration> server_config =
      aos::configuration::ReadConfig(
          "aos/network/message_bridge_test_server_config.json");
  aos::FlatbufferDetachedBuffer<aos::Configuration> client_config =
      aos::configuration::ReadConfig(
          "aos/network/message_bridge_test_client_config.json");

  FLAGS_application_name = "pi1_message_bridge_server";
  // Force ourselves to be "raspberrypi" and allocate everything.
  FLAGS_override_hostname = "raspberrypi";
  aos::ShmEventLoop pi1_server_event_loop(&server_config.message());
  MessageBridgeServer pi1_message_bridge_server(&pi1_server_event_loop);

  FLAGS_application_name = "pi1_message_bridge_client";
  aos::ShmEventLoop pi1_client_event_loop(&server_config.message());
  MessageBridgeClient pi1_message_bridge_client(&pi1_client_event_loop);

  // And build the app which sends the pings.
  FLAGS_application_name = "ping";
  aos::ShmEventLoop ping_event_loop(&server_config.message());
  aos::Sender<examples::Ping> ping_sender =
      ping_event_loop.MakeSender<examples::Ping>("/test");

  // Now do it for "raspberrypi2", the client.
  FLAGS_application_name = "pi2_message_bridge_client";
  FLAGS_override_hostname = "raspberrypi2";
  aos::ShmEventLoop pi2_client_event_loop(&client_config.message());
  MessageBridgeClient pi2_message_bridge_client(&pi2_client_event_loop);

  FLAGS_application_name = "pi2_message_bridge_server";
  aos::ShmEventLoop pi2_server_event_loop(&client_config.message());
  MessageBridgeServer pi2_message_bridge_server(&pi2_server_event_loop);

  // And build the app which sends the pongs.
  FLAGS_application_name = "pong";
  aos::ShmEventLoop pong_event_loop(&client_config.message());

  // And build the app for testing.
  FLAGS_application_name = "test";
  aos::ShmEventLoop test_event_loop(&client_config.message());

  aos::Fetcher<ClientStatistics> client_statistics_fetcher =
      test_event_loop.MakeFetcher<ClientStatistics>("/aos");

  // Count the pongs.
  int pong_count = 0;
  pong_event_loop.MakeWatcher(
      "/test2", [&pong_count](const examples::Ping &ping) {
        ++pong_count;
        LOG(INFO) << "Got ping back " << FlatbufferToJson(&ping);
      });

  FLAGS_override_hostname = "";

  // Wait until we are connected, then send.
  int ping_count = 0;
  int pi1_server_statistics_count = 0;
  ping_event_loop.MakeWatcher(
      "/aos/pi1",
      [&ping_count, &pi2_client_event_loop, &ping_sender,
       &pi1_server_statistics_count](const ServerStatistics &stats) {
        LOG(INFO) << FlatbufferToJson(&stats);

        ASSERT_TRUE(stats.has_connections());
        EXPECT_EQ(stats.connections()->size(), 1);

        bool connected = false;
        for (const ServerConnection *connection : *stats.connections()) {
          // Confirm that we are estimating the server time offset correctly. It
          // should be about 0 since we are on the same machine here.
          if (connection->has_monotonic_offset()) {
            EXPECT_LT(chrono::nanoseconds(connection->monotonic_offset()),
                      chrono::milliseconds(1));
            EXPECT_GT(chrono::nanoseconds(connection->monotonic_offset()),
                      chrono::milliseconds(-1));
            ++pi1_server_statistics_count;
          }

          if (connection->node()->name()->string_view() ==
              pi2_client_event_loop.node()->name()->string_view()) {
            if (connection->state() == State::CONNECTED) {
              connected = true;
            }
          }
        }

        if (connected) {
          LOG(INFO) << "Connected!  Sent ping.";
          auto builder = ping_sender.MakeBuilder();
          examples::Ping::Builder ping_builder =
              builder.MakeBuilder<examples::Ping>();
          ping_builder.add_value(ping_count + 971);
          builder.Send(ping_builder.Finish());
          ++ping_count;
        }
      });

  // Confirm both client and server statistics messages have decent offsets in
  // them.
  int pi2_server_statistics_count = 0;
  pong_event_loop.MakeWatcher("/aos/pi2", [&pi2_server_statistics_count](
                                              const ServerStatistics &stats) {
    LOG(INFO) << FlatbufferToJson(&stats);
    for (const ServerConnection *connection : *stats.connections()) {
      if (connection->has_monotonic_offset()) {
        ++pi2_server_statistics_count;
        // Confirm that we are estimating the server time offset correctly. It
        // should be about 0 since we are on the same machine here.
        EXPECT_LT(chrono::nanoseconds(connection->monotonic_offset()),
                  chrono::milliseconds(1));
        EXPECT_GT(chrono::nanoseconds(connection->monotonic_offset()),
                  chrono::milliseconds(-1));
      }
    }
  });

  int pi1_client_statistics_count = 0;
  ping_event_loop.MakeWatcher(
      "/aos/pi1", [&pi1_client_statistics_count](const ClientStatistics &stats) {
        LOG(INFO) << FlatbufferToJson(&stats);

        for (const ClientConnection *connection : *stats.connections()) {
          if (connection->has_monotonic_offset()) {
            ++pi1_client_statistics_count;
            // It takes at least 10 microseconds to send a message between the
            // client and server.  The min (filtered) time shouldn't be over 10
            // milliseconds on localhost.  This might have to bump up if this is
            // proving flaky.
            EXPECT_LT(chrono::nanoseconds(connection->monotonic_offset()),
                      chrono::milliseconds(10));
            EXPECT_GT(chrono::nanoseconds(connection->monotonic_offset()),
                      chrono::microseconds(10));
          }
        }
      });

  int pi2_client_statistics_count = 0;
  pong_event_loop.MakeWatcher("/aos/pi2", [&pi2_client_statistics_count](
                                              const ClientStatistics &stats) {
    LOG(INFO) << FlatbufferToJson(&stats);

    for (const ClientConnection *connection : *stats.connections()) {
      if (connection->has_monotonic_offset()) {
        ++pi2_client_statistics_count;
        EXPECT_LT(chrono::nanoseconds(connection->monotonic_offset()),
                  chrono::milliseconds(10));
        EXPECT_GT(chrono::nanoseconds(connection->monotonic_offset()),
                  chrono::microseconds(10));
      }
    }
  });

  ping_event_loop.MakeWatcher("/aos/pi1", [](const Timestamp &timestamp) {
    EXPECT_TRUE(timestamp.has_offsets());
    LOG(INFO) << FlatbufferToJson(&timestamp);
  });
  pong_event_loop.MakeWatcher("/aos/pi2", [](const Timestamp &timestamp) {
    EXPECT_TRUE(timestamp.has_offsets());
    LOG(INFO) << FlatbufferToJson(&timestamp);
  });

  // Run for 5 seconds to make sure we have time to estimate the offset.
  aos::TimerHandler *quit = ping_event_loop.AddTimer(
      [&ping_event_loop]() { ping_event_loop.Exit(); });
  ping_event_loop.OnRun([quit, &ping_event_loop]() {
    // Stop between timestamps, not exactly on them.
    quit->Setup(ping_event_loop.monotonic_now() + chrono::milliseconds(5050));
  });

  // Start everything up.  Pong is the only thing we don't know how to wait on,
  // so start it first.
  std::thread pong_thread([&pong_event_loop]() { pong_event_loop.Run(); });

  std::thread pi1_server_thread(
      [&pi1_server_event_loop]() { pi1_server_event_loop.Run(); });
  std::thread pi1_client_thread(
      [&pi1_client_event_loop]() { pi1_client_event_loop.Run(); });
  std::thread pi2_client_thread(
      [&pi2_client_event_loop]() { pi2_client_event_loop.Run(); });
  std::thread pi2_server_thread(
      [&pi2_server_event_loop]() { pi2_server_event_loop.Run(); });

  // And go!
  ping_event_loop.Run();

  // Shut everyone else down
  pi1_server_event_loop.Exit();
  pi1_client_event_loop.Exit();
  pi2_client_event_loop.Exit();
  pi2_server_event_loop.Exit();
  pong_event_loop.Exit();
  pi1_server_thread.join();
  pi1_client_thread.join();
  pi2_client_thread.join();
  pi2_server_thread.join();
  pong_thread.join();

  // Make sure we sent something.
  EXPECT_GE(ping_count, 1);
  // And got something back.
  EXPECT_GE(pong_count, 1);

  // Confirm that we are estimating a monotonic offset on the client.
  ASSERT_TRUE(client_statistics_fetcher.Fetch());

  EXPECT_EQ(client_statistics_fetcher->connections()->size(), 1u);
  EXPECT_EQ(client_statistics_fetcher->connections()
                ->Get(0)
                ->node()
                ->name()
                ->string_view(),
            "pi1");

  // Make sure the offset in one direction is less than a second.
  EXPECT_GT(
      client_statistics_fetcher->connections()->Get(0)->monotonic_offset(), 0);
  EXPECT_LT(
      client_statistics_fetcher->connections()->Get(0)->monotonic_offset(),
      1000000000);

  EXPECT_GE(pi1_server_statistics_count, 2);
  EXPECT_GE(pi2_server_statistics_count, 2);
  EXPECT_GE(pi1_client_statistics_count, 2);
  EXPECT_GE(pi2_client_statistics_count, 2);

  // TODO(austin): Need 2 servers going so we can do the round trip offset
  // estimation.
}

}  // namespace testing
}  // namespace message_bridge
}  // namespace aos
