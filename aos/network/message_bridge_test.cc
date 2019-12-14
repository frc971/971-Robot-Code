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
  aos::ShmEventLoop server_event_loop(&server_config.message());
  MessageBridgeServer message_bridge_server(&server_event_loop);

  // And build the app which sends the pings.
  FLAGS_application_name = "ping";
  aos::ShmEventLoop ping_event_loop(&server_config.message());
  aos::Sender<examples::Ping> ping_sender =
      ping_event_loop.MakeSender<examples::Ping>("/test");

  // Now do it for "raspberrypi2", the client.
  FLAGS_application_name = "pi2_message_bridge_client";
  FLAGS_override_hostname = "raspberrypi2";
  aos::ShmEventLoop client_event_loop(&client_config.message());
  MessageBridgeClient message_bridge_client(&client_event_loop);

  // And build the app which sends the pongs.
  FLAGS_application_name = "pong";
  aos::ShmEventLoop pong_event_loop(&client_config.message());

  // Count the pongs.
  int pong_count = 0;
  pong_event_loop.MakeWatcher(
      "/test2", [&pong_count, &ping_event_loop](const examples::Ping &ping) {
        ++pong_count;
        LOG(INFO) << "Got ping back " << FlatbufferToJson(&ping);
        if (pong_count >= 2) {
          LOG(INFO) << "That's enough bailing early.";
          // And Exit is async safe, so thread safe is easy.
          ping_event_loop.Exit();
        }
      });

  FLAGS_override_hostname = "";

  // Start everything up.  Pong is the only thing we don't know how to wait on,
  // so start it first.
  std::thread pong_thread([&pong_event_loop]() { pong_event_loop.Run(); });

  std::thread server_thread(
      [&server_event_loop]() { server_event_loop.Run(); });
  std::thread client_thread(
      [&client_event_loop]() { client_event_loop.Run(); });

  // Wait until we are connected, then send.
  int ping_count = 0;
  ping_event_loop.MakeWatcher(
      "/aos/pi1", [&ping_count, &client_event_loop,
                   &ping_sender](const ServerStatistics &stats) {
        LOG(INFO) << FlatbufferToJson(&stats);

        ASSERT_TRUE(stats.has_connections());
        EXPECT_EQ(stats.connections()->size(), 1);

        bool connected = false;
        for (const ServerConnection *connection : *stats.connections()) {
          if (connection->node()->name()->string_view() ==
              client_event_loop.node()->name()->string_view()) {
            if (connection->state() == State::CONNECTED) {
              connected = true;
            }
            break;
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

  // Time ourselves out after a while if Pong doesn't do it for us.
  aos::TimerHandler *quit = ping_event_loop.AddTimer(
      [&ping_event_loop]() { ping_event_loop.Exit(); });
  ping_event_loop.OnRun([quit, &ping_event_loop]() {
    quit->Setup(ping_event_loop.monotonic_now() + chrono::seconds(10));
  });


  // And go!
  ping_event_loop.Run();

  // Shut everyone else down
  server_event_loop.Exit();
  client_event_loop.Exit();
  pong_event_loop.Exit();
  server_thread.join();
  client_thread.join();
  pong_thread.join();

  // Make sure we sent something.
  EXPECT_GE(ping_count, 1);
  // And got something back.
  EXPECT_GE(pong_count, 1);
}

}  // namespace testing
}  // namespace message_bridge
}  // namespace aos
