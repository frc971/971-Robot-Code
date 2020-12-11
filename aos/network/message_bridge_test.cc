#include "gtest/gtest.h"

#include <chrono>
#include <thread>

#include "absl/strings/str_cat.h"
#include "aos/event.h"
#include "aos/events/ping_generated.h"
#include "aos/events/pong_generated.h"
#include "aos/network/message_bridge_client_lib.h"
#include "aos/network/message_bridge_server_lib.h"
#include "aos/network/team_number.h"
#include "aos/util/file.h"

namespace aos {
void SetShmBase(const std::string_view base);

namespace message_bridge {
namespace testing {

namespace chrono = std::chrono;

std::string ShmBase(const std::string_view node) {
  const char *tmpdir_c_str = getenv("TEST_TMPDIR");
  if (tmpdir_c_str != nullptr) {
    return absl::StrCat(tmpdir_c_str, "/", node);
  } else {
    return absl::StrCat("/dev/shm/", node);
  }
}

void DoSetShmBase(const std::string_view node) {
  aos::SetShmBase(ShmBase(node));
}

class MessageBridgeTest : public ::testing::Test {
 public:
  MessageBridgeTest()
      : pi1_config(aos::configuration::ReadConfig(
            "aos/network/message_bridge_test_server_config.json")),
        pi2_config(aos::configuration::ReadConfig(
            "aos/network/message_bridge_test_client_config.json")) {
    util::UnlinkRecursive(ShmBase("pi1"));
    util::UnlinkRecursive(ShmBase("pi2"));
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> pi1_config;
  aos::FlatbufferDetachedBuffer<aos::Configuration> pi2_config;
};

// Test that we can send a ping message over sctp and receive it.
TEST_F(MessageBridgeTest, PingPong) {
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
  DoSetShmBase("pi1");
  FLAGS_application_name = "pi1_message_bridge_server";
  // Force ourselves to be "raspberrypi" and allocate everything.
  FLAGS_override_hostname = "raspberrypi";

  aos::ShmEventLoop pi1_server_event_loop(&pi1_config.message());
  MessageBridgeServer pi1_message_bridge_server(&pi1_server_event_loop);

  FLAGS_application_name = "pi1_message_bridge_client";
  aos::ShmEventLoop pi1_client_event_loop(&pi1_config.message());
  MessageBridgeClient pi1_message_bridge_client(&pi1_client_event_loop);

  // And build the app which sends the pings.
  FLAGS_application_name = "ping";
  aos::ShmEventLoop ping_event_loop(&pi1_config.message());
  aos::Sender<examples::Ping> ping_sender =
      ping_event_loop.MakeSender<examples::Ping>("/test");

  aos::ShmEventLoop pi1_test_event_loop(&pi1_config.message());
  aos::Fetcher<RemoteMessage> message_header_fetcher1 =
      pi1_test_event_loop.MakeFetcher<RemoteMessage>(
          "/pi1/aos/remote_timestamps/pi2");

  // Fetchers for confirming the remote timestamps made it.
  aos::Fetcher<examples::Ping> ping_on_pi1_fetcher =
      ping_event_loop.MakeFetcher<examples::Ping>("/test");
  aos::Fetcher<Timestamp> pi1_on_pi1_timestamp_fetcher =
      ping_event_loop.MakeFetcher<Timestamp>("/aos");

  // Now do it for "raspberrypi2", the client.
  FLAGS_application_name = "pi2_message_bridge_client";
  FLAGS_override_hostname = "raspberrypi2";
  DoSetShmBase("pi2");

  aos::ShmEventLoop pi2_client_event_loop(&pi2_config.message());
  MessageBridgeClient pi2_message_bridge_client(&pi2_client_event_loop);

  FLAGS_application_name = "pi2_message_bridge_server";
  aos::ShmEventLoop pi2_server_event_loop(&pi2_config.message());
  MessageBridgeServer pi2_message_bridge_server(&pi2_server_event_loop);

  // And build the app which sends the pongs.
  FLAGS_application_name = "pong";
  aos::ShmEventLoop pong_event_loop(&pi2_config.message());

  // And build the app for testing.
  FLAGS_application_name = "test";
  aos::ShmEventLoop test_event_loop(&pi2_config.message());

  aos::Fetcher<ClientStatistics> client_statistics_fetcher =
      test_event_loop.MakeFetcher<ClientStatistics>("/aos");
  aos::Fetcher<RemoteMessage> message_header_fetcher2 =
      test_event_loop.MakeFetcher<RemoteMessage>(
          "/pi2/aos/remote_timestamps/pi1");

  // Event loop for fetching data delivered to pi2 from pi1 to match up
  // messages.
  aos::ShmEventLoop delivered_messages_event_loop(&pi2_config.message());
  aos::Fetcher<Timestamp> pi1_on_pi2_timestamp_fetcher =
      delivered_messages_event_loop.MakeFetcher<Timestamp>("/pi1/aos");
  aos::Fetcher<examples::Ping> ping_on_pi2_fetcher =
      delivered_messages_event_loop.MakeFetcher<examples::Ping>("/test");
  EXPECT_FALSE(ping_on_pi2_fetcher.Fetch());
  EXPECT_FALSE(pi1_on_pi2_timestamp_fetcher.Fetch());

  // Count the pongs.
  int pong_count = 0;
  pong_event_loop.MakeWatcher(
      "/test", [&pong_count](const examples::Ping &ping) {
        ++pong_count;
        VLOG(1) << "Got ping back " << FlatbufferToJson(&ping);
      });

  FLAGS_override_hostname = "";

  // Wait until we are connected, then send.
  int ping_count = 0;
  int pi1_server_statistics_count = 0;
  ping_event_loop.MakeWatcher(
      "/pi1/aos",
      [&ping_count, &pi2_client_event_loop, &ping_sender,
       &pi1_server_statistics_count](const ServerStatistics &stats) {
        VLOG(1) << "/pi1/aos ServerStatistics " << FlatbufferToJson(&stats);

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
              EXPECT_TRUE(connection->has_boot_uuid());
              connected = true;
            }
          }
        }

        if (connected) {
          VLOG(1) << "Connected!  Sent ping.";
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
  pong_event_loop.MakeWatcher("/pi2/aos", [&pi2_server_statistics_count](
                                              const ServerStatistics &stats) {
    VLOG(1) << "/pi2/aos ServerStatistics " << FlatbufferToJson(&stats);
    for (const ServerConnection *connection : *stats.connections()) {
      if (connection->has_monotonic_offset()) {
        ++pi2_server_statistics_count;
        // Confirm that we are estimating the server time offset correctly. It
        // should be about 0 since we are on the same machine here.
        EXPECT_LT(chrono::nanoseconds(connection->monotonic_offset()),
                  chrono::milliseconds(1));
        EXPECT_GT(chrono::nanoseconds(connection->monotonic_offset()),
                  chrono::milliseconds(-1));
        EXPECT_TRUE(connection->has_boot_uuid());
      }
    }
  });

  int pi1_client_statistics_count = 0;
  ping_event_loop.MakeWatcher("/pi1/aos", [&pi1_client_statistics_count](
                                              const ClientStatistics &stats) {
    VLOG(1) << "/pi1/aos ClientStatistics " << FlatbufferToJson(&stats);

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
  pong_event_loop.MakeWatcher("/pi2/aos", [&pi2_client_statistics_count](
                                              const ClientStatistics &stats) {
    VLOG(1) << "/pi2/aos ClientStatistics " << FlatbufferToJson(&stats);

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

  ping_event_loop.MakeWatcher("/pi1/aos", [](const Timestamp &timestamp) {
    EXPECT_TRUE(timestamp.has_offsets());
    VLOG(1) << "/pi1/aos Timestamp " << FlatbufferToJson(&timestamp);
  });
  pong_event_loop.MakeWatcher("/pi2/aos", [](const Timestamp &timestamp) {
    EXPECT_TRUE(timestamp.has_offsets());
    VLOG(1) << "/pi2/aos Timestamp " << FlatbufferToJson(&timestamp);
  });

  // Run for 5 seconds to make sure we have time to estimate the offset.
  aos::TimerHandler *quit = ping_event_loop.AddTimer(
      [&ping_event_loop]() { ping_event_loop.Exit(); });
  ping_event_loop.OnRun([quit, &ping_event_loop]() {
    // Stop between timestamps, not exactly on them.
    quit->Setup(ping_event_loop.monotonic_now() + chrono::milliseconds(5050));
  });

  // Find the channel index for both the /pi1/aos Timestamp channel and Ping
  // channel.
  const size_t pi1_timestamp_channel = configuration::ChannelIndex(
      pong_event_loop.configuration(), pi1_on_pi2_timestamp_fetcher.channel());
  const size_t ping_timestamp_channel =
      configuration::ChannelIndex(delivered_messages_event_loop.configuration(),
                                  ping_on_pi2_fetcher.channel());

  for (const Channel *channel : *ping_event_loop.configuration()->channels()) {
    VLOG(1) << "Channel "
            << configuration::ChannelIndex(ping_event_loop.configuration(),
                                           channel)
            << " " << configuration::CleanedChannelToString(channel);
  }

  // For each remote timestamp we get back, confirm that it is either a ping
  // message, or a timestamp we sent out.  Also confirm that the timestamps are
  // correct.
  ping_event_loop.MakeWatcher(
      "/pi1/aos/remote_timestamps/pi2",
      [pi1_timestamp_channel, ping_timestamp_channel, &ping_on_pi2_fetcher,
       &ping_on_pi1_fetcher, &pi1_on_pi2_timestamp_fetcher,
       &pi1_on_pi1_timestamp_fetcher](const RemoteMessage &header) {
        VLOG(1) << "/pi1/aos/remote_timestamps/pi2 RemoteMessage "
                << aos::FlatbufferToJson(&header);

        EXPECT_TRUE(header.has_boot_uuid());

        const aos::monotonic_clock::time_point header_monotonic_sent_time(
            chrono::nanoseconds(header.monotonic_sent_time()));
        const aos::realtime_clock::time_point header_realtime_sent_time(
            chrono::nanoseconds(header.realtime_sent_time()));
        const aos::monotonic_clock::time_point header_monotonic_remote_time(
            chrono::nanoseconds(header.monotonic_remote_time()));
        const aos::realtime_clock::time_point header_realtime_remote_time(
            chrono::nanoseconds(header.realtime_remote_time()));

        const Context *pi1_context = nullptr;
        const Context *pi2_context = nullptr;

        if (header.channel_index() == pi1_timestamp_channel) {
          // Find the forwarded message.
          while (pi1_on_pi2_timestamp_fetcher.context().monotonic_event_time <
                 header_monotonic_sent_time) {
            ASSERT_TRUE(pi1_on_pi2_timestamp_fetcher.FetchNext());
          }

          // And the source message.
          while (pi1_on_pi1_timestamp_fetcher.context().monotonic_event_time <
                 header_monotonic_remote_time) {
            ASSERT_TRUE(pi1_on_pi1_timestamp_fetcher.FetchNext());
          }

          pi1_context = &pi1_on_pi1_timestamp_fetcher.context();
          pi2_context = &pi1_on_pi2_timestamp_fetcher.context();
        } else if (header.channel_index() == ping_timestamp_channel) {
          // Find the forwarded message.
          while (ping_on_pi2_fetcher.context().monotonic_event_time <
                 header_monotonic_sent_time) {
            ASSERT_TRUE(ping_on_pi2_fetcher.FetchNext());
          }

          // And the source message.
          while (ping_on_pi1_fetcher.context().monotonic_event_time <
                 header_monotonic_remote_time) {
            ASSERT_TRUE(ping_on_pi1_fetcher.FetchNext());
          }

          pi1_context = &ping_on_pi1_fetcher.context();
          pi2_context = &ping_on_pi2_fetcher.context();
        } else {
          LOG(FATAL) << "Unknown channel";
        }

        // Confirm the forwarded message has matching timestamps to the
        // timestamps we got back.
        EXPECT_EQ(pi2_context->queue_index, header.queue_index());
        EXPECT_EQ(pi2_context->monotonic_event_time,
                  header_monotonic_sent_time);
        EXPECT_EQ(pi2_context->realtime_event_time, header_realtime_sent_time);
        EXPECT_EQ(pi2_context->realtime_remote_time,
                  header_realtime_remote_time);
        EXPECT_EQ(pi2_context->monotonic_remote_time,
                  header_monotonic_remote_time);

        // Confirm the forwarded message also matches the source message.
        EXPECT_EQ(pi1_context->queue_index, header.queue_index());
        EXPECT_EQ(pi1_context->monotonic_event_time,
                  header_monotonic_remote_time);
        EXPECT_EQ(pi1_context->realtime_event_time,
                  header_realtime_remote_time);
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

  // Confirm we got timestamps back!
  EXPECT_TRUE(message_header_fetcher1.Fetch());
  EXPECT_TRUE(message_header_fetcher2.Fetch());
}

// Test that the client disconnecting triggers the server offsets on both sides
// to clear.
TEST_F(MessageBridgeTest, ClientRestart) {
  // This is rather annoying to set up.  We need to start up a client and
  // server, on the same node, but get them to think that they are on different
  // nodes.
  //
  // We need the client to not post directly to "/test" like it would in a
  // real system, otherwise we will re-send the ping message... So, use an
  // application specific map to have the client post somewhere else.
  //
  // To top this all off, each of these needs to be done with a ShmEventLoop,
  // which needs to run in a separate thread...  And it is really hard to get
  // everything started up reliably.  So just be super generous on timeouts and
  // hope for the best.  We can be more generous in the future if we need to.
  //
  // We are faking the application names by passing in --application_name=foo
  FLAGS_application_name = "pi1_message_bridge_server";
  // Force ourselves to be "raspberrypi" and allocate everything.
  FLAGS_override_hostname = "raspberrypi";
  DoSetShmBase("pi1");
  aos::ShmEventLoop pi1_server_event_loop(&pi1_config.message());
  MessageBridgeServer pi1_message_bridge_server(&pi1_server_event_loop);

  FLAGS_application_name = "pi1_message_bridge_client";
  aos::ShmEventLoop pi1_client_event_loop(&pi1_config.message());
  MessageBridgeClient pi1_message_bridge_client(&pi1_client_event_loop);

  // And build the app for testing.
  FLAGS_application_name = "test1";
  aos::ShmEventLoop pi1_test_event_loop(&pi1_config.message());
  aos::Fetcher<ServerStatistics> pi1_server_statistics_fetcher =
      pi1_test_event_loop.MakeFetcher<ServerStatistics>("/pi1/aos");

  // Now do it for "raspberrypi2", the client.
  FLAGS_override_hostname = "raspberrypi2";
  DoSetShmBase("pi2");
  FLAGS_application_name = "pi2_message_bridge_server";
  aos::ShmEventLoop pi2_server_event_loop(&pi2_config.message());
  MessageBridgeServer pi2_message_bridge_server(&pi2_server_event_loop);

  // And build the app for testing.
  FLAGS_application_name = "test2";
  aos::ShmEventLoop pi2_test_event_loop(&pi2_config.message());
  aos::Fetcher<ServerStatistics> pi2_server_statistics_fetcher =
      pi2_test_event_loop.MakeFetcher<ServerStatistics>("/pi2/aos");

  // Wait until we are connected, then send.
  pi1_test_event_loop.MakeWatcher(
      "/pi1/aos", [](const ServerStatistics &stats) {
        VLOG(1) << "/pi1/aos ServerStatistics " << FlatbufferToJson(&stats);
      });

  pi2_test_event_loop.MakeWatcher(
      "/pi2/aos", [](const ServerStatistics &stats) {
        VLOG(1) << "/pi2/aos ServerStatistics " << FlatbufferToJson(&stats);
      });

  pi1_test_event_loop.MakeWatcher(
      "/pi1/aos", [](const ClientStatistics &stats) {
        VLOG(1) << "/pi1/aos ClientStatistics " << FlatbufferToJson(&stats);
      });

  pi2_test_event_loop.MakeWatcher(
      "/pi2/aos", [](const ClientStatistics &stats) {
        VLOG(1) << "/pi2/aos ClientStatistics " << FlatbufferToJson(&stats);
      });

  pi1_test_event_loop.MakeWatcher("/pi1/aos", [](const Timestamp &timestamp) {
    VLOG(1) << "/pi1/aos Timestamp " << FlatbufferToJson(&timestamp);
  });
  pi2_test_event_loop.MakeWatcher("/pi2/aos", [](const Timestamp &timestamp) {
    VLOG(1) << "/pi2/aos Timestamp " << FlatbufferToJson(&timestamp);
  });

  // Start everything up.  Pong is the only thing we don't know how to wait on,
  // so start it first.
  std::thread pi1_test_thread(
      [&pi1_test_event_loop]() { pi1_test_event_loop.Run(); });
  std::thread pi2_test_thread(
      [&pi2_test_event_loop]() { pi2_test_event_loop.Run(); });

  std::thread pi1_server_thread(
      [&pi1_server_event_loop]() { pi1_server_event_loop.Run(); });
  std::thread pi1_client_thread(
      [&pi1_client_event_loop]() { pi1_client_event_loop.Run(); });
  std::thread pi2_server_thread(
      [&pi2_server_event_loop]() { pi2_server_event_loop.Run(); });

  {
    FLAGS_application_name = "pi2_message_bridge_client";
    aos::ShmEventLoop pi2_client_event_loop(&pi2_config.message());
    MessageBridgeClient pi2_message_bridge_client(&pi2_client_event_loop);

    // Run for 5 seconds to make sure we have time to estimate the offset.
    aos::TimerHandler *const quit = pi2_client_event_loop.AddTimer(
        [&pi2_client_event_loop]() { pi2_client_event_loop.Exit(); });
    pi2_client_event_loop.OnRun([quit, &pi2_client_event_loop]() {
      // Stop between timestamps, not exactly on them.
      quit->Setup(pi2_client_event_loop.monotonic_now() +
                  chrono::milliseconds(3050));
    });

    // And go!
    pi2_client_event_loop.Run();

    // Now confirm we are synchronized.
    EXPECT_TRUE(pi1_server_statistics_fetcher.Fetch());
    EXPECT_TRUE(pi2_server_statistics_fetcher.Fetch());

    const ServerConnection *const pi1_connection =
        pi1_server_statistics_fetcher->connections()->Get(0);
    const ServerConnection *const pi2_connection =
        pi2_server_statistics_fetcher->connections()->Get(0);

    EXPECT_EQ(pi1_connection->state(), State::CONNECTED);
    EXPECT_TRUE(pi1_connection->has_monotonic_offset());
    EXPECT_LT(chrono::nanoseconds(pi1_connection->monotonic_offset()),
              chrono::milliseconds(1));
    EXPECT_GT(chrono::nanoseconds(pi1_connection->monotonic_offset()),
              chrono::milliseconds(-1));
    EXPECT_TRUE(pi1_connection->has_boot_uuid());

    EXPECT_EQ(pi2_connection->state(), State::CONNECTED);
    EXPECT_TRUE(pi2_connection->has_monotonic_offset());
    EXPECT_LT(chrono::nanoseconds(pi2_connection->monotonic_offset()),
              chrono::milliseconds(1));
    EXPECT_GT(chrono::nanoseconds(pi2_connection->monotonic_offset()),
              chrono::milliseconds(-1));
    EXPECT_TRUE(pi2_connection->has_boot_uuid());
  }

  std::this_thread::sleep_for(std::chrono::seconds(2));

  {
    // Now confirm we are un-synchronized.
    EXPECT_TRUE(pi1_server_statistics_fetcher.Fetch());
    EXPECT_TRUE(pi2_server_statistics_fetcher.Fetch());
    const ServerConnection *const pi1_connection =
        pi1_server_statistics_fetcher->connections()->Get(0);
    const ServerConnection *const pi2_connection =
        pi2_server_statistics_fetcher->connections()->Get(0);

    EXPECT_EQ(pi1_connection->state(), State::DISCONNECTED);
    EXPECT_FALSE(pi1_connection->has_monotonic_offset());
    EXPECT_FALSE(pi1_connection->has_boot_uuid());
    EXPECT_EQ(pi2_connection->state(), State::CONNECTED);
    EXPECT_FALSE(pi2_connection->has_monotonic_offset());
    EXPECT_TRUE(pi2_connection->has_boot_uuid());
  }

  {
    FLAGS_application_name = "pi2_message_bridge_client";
    aos::ShmEventLoop pi2_client_event_loop(&pi2_config.message());
    MessageBridgeClient pi2_message_bridge_client(&pi2_client_event_loop);

    // Run for 5 seconds to make sure we have time to estimate the offset.
    aos::TimerHandler *const quit = pi2_client_event_loop.AddTimer(
        [&pi2_client_event_loop]() { pi2_client_event_loop.Exit(); });
    pi2_client_event_loop.OnRun([quit, &pi2_client_event_loop]() {
      // Stop between timestamps, not exactly on them.
      quit->Setup(pi2_client_event_loop.monotonic_now() +
                  chrono::milliseconds(3050));
    });

    // And go!
    pi2_client_event_loop.Run();

    EXPECT_TRUE(pi1_server_statistics_fetcher.Fetch());
    EXPECT_TRUE(pi2_server_statistics_fetcher.Fetch());

    // Now confirm we are synchronized again.
    const ServerConnection *const pi1_connection =
        pi1_server_statistics_fetcher->connections()->Get(0);
    const ServerConnection *const pi2_connection =
        pi2_server_statistics_fetcher->connections()->Get(0);

    EXPECT_EQ(pi1_connection->state(), State::CONNECTED);
    EXPECT_TRUE(pi1_connection->has_monotonic_offset());
    EXPECT_LT(chrono::nanoseconds(pi1_connection->monotonic_offset()),
              chrono::milliseconds(1));
    EXPECT_GT(chrono::nanoseconds(pi1_connection->monotonic_offset()),
              chrono::milliseconds(-1));
    EXPECT_TRUE(pi1_connection->has_boot_uuid());

    EXPECT_EQ(pi2_connection->state(), State::CONNECTED);
    EXPECT_TRUE(pi2_connection->has_monotonic_offset());
    EXPECT_LT(chrono::nanoseconds(pi2_connection->monotonic_offset()),
              chrono::milliseconds(1));
    EXPECT_GT(chrono::nanoseconds(pi2_connection->monotonic_offset()),
              chrono::milliseconds(-1));
    EXPECT_TRUE(pi2_connection->has_boot_uuid());
  }

  // Shut everyone else down
  pi1_server_event_loop.Exit();
  pi1_client_event_loop.Exit();
  pi2_server_event_loop.Exit();
  pi1_test_event_loop.Exit();
  pi2_test_event_loop.Exit();
  pi1_server_thread.join();
  pi1_client_thread.join();
  pi2_server_thread.join();
  pi1_test_thread.join();
  pi2_test_thread.join();
}

// Test that the server disconnecting triggers the server offsets on the other
// side to clear, along with the other client.
TEST_F(MessageBridgeTest, ServerRestart) {
  // This is rather annoying to set up.  We need to start up a client and
  // server, on the same node, but get them to think that they are on different
  // nodes.
  //
  // We need the client to not post directly to "/test" like it would in a
  // real system, otherwise we will re-send the ping message... So, use an
  // application specific map to have the client post somewhere else.
  //
  // To top this all off, each of these needs to be done with a ShmEventLoop,
  // which needs to run in a separate thread...  And it is really hard to get
  // everything started up reliably.  So just be super generous on timeouts and
  // hope for the best.  We can be more generous in the future if we need to.
  //
  // We are faking the application names by passing in --application_name=foo
  FLAGS_application_name = "pi1_message_bridge_server";
  // Force ourselves to be "raspberrypi" and allocate everything.
  FLAGS_override_hostname = "raspberrypi";
  DoSetShmBase("pi1");
  aos::ShmEventLoop pi1_server_event_loop(&pi1_config.message());
  MessageBridgeServer pi1_message_bridge_server(&pi1_server_event_loop);

  FLAGS_application_name = "pi1_message_bridge_client";
  aos::ShmEventLoop pi1_client_event_loop(&pi1_config.message());
  MessageBridgeClient pi1_message_bridge_client(&pi1_client_event_loop);

  // And build the app for testing.
  FLAGS_application_name = "test1";
  aos::ShmEventLoop pi1_test_event_loop(&pi1_config.message());
  aos::Fetcher<ServerStatistics> pi1_server_statistics_fetcher =
      pi1_test_event_loop.MakeFetcher<ServerStatistics>("/pi1/aos");
  aos::Fetcher<ClientStatistics> pi1_client_statistics_fetcher =
      pi1_test_event_loop.MakeFetcher<ClientStatistics>("/pi1/aos");

  // Now do it for "raspberrypi2", the client.
  FLAGS_override_hostname = "raspberrypi2";
  DoSetShmBase("pi2");
  FLAGS_application_name = "pi2_message_bridge_client";
  aos::ShmEventLoop pi2_client_event_loop(&pi2_config.message());
  MessageBridgeClient pi2_message_bridge_client(&pi2_client_event_loop);

  // And build the app for testing.
  FLAGS_application_name = "test2";
  aos::ShmEventLoop pi2_test_event_loop(&pi2_config.message());
  aos::Fetcher<ServerStatistics> pi2_server_statistics_fetcher =
      pi2_test_event_loop.MakeFetcher<ServerStatistics>("/pi2/aos");

  // Wait until we are connected, then send.
  pi1_test_event_loop.MakeWatcher(
      "/pi1/aos", [](const ServerStatistics &stats) {
        VLOG(1) << "/pi1/aos ServerStatistics " << FlatbufferToJson(&stats);
      });

  // Confirm both client and server statistics messages have decent offsets in
  // them.
  pi2_test_event_loop.MakeWatcher(
      "/pi2/aos", [](const ServerStatistics &stats) {
        VLOG(1) << "/pi2/aos ServerStatistics " << FlatbufferToJson(&stats);
      });

  pi1_test_event_loop.MakeWatcher(
      "/pi1/aos", [](const ClientStatistics &stats) {
        VLOG(1) << "/pi1/aos ClientStatistics " << FlatbufferToJson(&stats);
      });

  pi2_test_event_loop.MakeWatcher(
      "/pi2/aos", [](const ClientStatistics &stats) {
        VLOG(1) << "/pi2/aos ClientStatistics " << FlatbufferToJson(&stats);
      });

  pi1_test_event_loop.MakeWatcher("/pi1/aos", [](const Timestamp &timestamp) {
    VLOG(1) << "/pi1/aos Timestamp " << FlatbufferToJson(&timestamp);
  });
  pi2_test_event_loop.MakeWatcher("/pi2/aos", [](const Timestamp &timestamp) {
    VLOG(1) << "/pi2/aos Timestamp " << FlatbufferToJson(&timestamp);
  });

  // Start everything up.  Pong is the only thing we don't know how to wait on,
  // so start it first.
  std::thread pi1_test_thread(
      [&pi1_test_event_loop]() { pi1_test_event_loop.Run(); });
  std::thread pi2_test_thread(
      [&pi2_test_event_loop]() { pi2_test_event_loop.Run(); });

  std::thread pi1_server_thread(
      [&pi1_server_event_loop]() { pi1_server_event_loop.Run(); });
  std::thread pi1_client_thread(
      [&pi1_client_event_loop]() { pi1_client_event_loop.Run(); });
  std::thread pi2_client_thread(
      [&pi2_client_event_loop]() { pi2_client_event_loop.Run(); });

  {
    FLAGS_application_name = "pi2_message_bridge_server";
    aos::ShmEventLoop pi2_server_event_loop(&pi2_config.message());
    MessageBridgeServer pi2_message_bridge_server(&pi2_server_event_loop);

    // Run for 5 seconds to make sure we have time to estimate the offset.
    aos::TimerHandler *const quit = pi2_server_event_loop.AddTimer(
        [&pi2_server_event_loop]() { pi2_server_event_loop.Exit(); });
    pi2_server_event_loop.OnRun([quit, &pi2_server_event_loop]() {
      // Stop between timestamps, not exactly on them.
      quit->Setup(pi2_server_event_loop.monotonic_now() +
                  chrono::milliseconds(3050));
    });

    // And go!
    pi2_server_event_loop.Run();

    // Now confirm we are synchronized.
    EXPECT_TRUE(pi1_server_statistics_fetcher.Fetch());
    EXPECT_TRUE(pi2_server_statistics_fetcher.Fetch());

    const ServerConnection *const pi1_connection =
        pi1_server_statistics_fetcher->connections()->Get(0);
    const ServerConnection *const pi2_connection =
        pi2_server_statistics_fetcher->connections()->Get(0);

    EXPECT_EQ(pi1_connection->state(), State::CONNECTED);
    EXPECT_TRUE(pi1_connection->has_monotonic_offset());
    EXPECT_LT(chrono::nanoseconds(pi1_connection->monotonic_offset()),
              chrono::milliseconds(1));
    EXPECT_GT(chrono::nanoseconds(pi1_connection->monotonic_offset()),
              chrono::milliseconds(-1));
    EXPECT_TRUE(pi1_connection->has_boot_uuid());

    EXPECT_EQ(pi2_connection->state(), State::CONNECTED);
    EXPECT_TRUE(pi2_connection->has_monotonic_offset());
    EXPECT_LT(chrono::nanoseconds(pi2_connection->monotonic_offset()),
              chrono::milliseconds(1));
    EXPECT_GT(chrono::nanoseconds(pi2_connection->monotonic_offset()),
              chrono::milliseconds(-1));
    EXPECT_TRUE(pi2_connection->has_boot_uuid());
  }

  std::this_thread::sleep_for(std::chrono::seconds(2));

  {
    // And confirm we are unsynchronized.
    EXPECT_TRUE(pi1_server_statistics_fetcher.Fetch());
    EXPECT_TRUE(pi1_client_statistics_fetcher.Fetch());

    const ServerConnection *const pi1_server_connection =
        pi1_server_statistics_fetcher->connections()->Get(0);
    const ClientConnection *const pi1_client_connection =
        pi1_client_statistics_fetcher->connections()->Get(0);

    EXPECT_EQ(pi1_server_connection->state(), State::CONNECTED);
    EXPECT_FALSE(pi1_server_connection->has_monotonic_offset());
    EXPECT_TRUE(pi1_server_connection->has_boot_uuid());
    EXPECT_EQ(pi1_client_connection->state(), State::DISCONNECTED);
    EXPECT_FALSE(pi1_client_connection->has_monotonic_offset());
  }

  {
    FLAGS_application_name = "pi2_message_bridge_server";
    aos::ShmEventLoop pi2_server_event_loop(&pi2_config.message());
    MessageBridgeServer pi2_message_bridge_server(&pi2_server_event_loop);

    // Run for 5 seconds to make sure we have time to estimate the offset.
    aos::TimerHandler *const quit = pi2_server_event_loop.AddTimer(
        [&pi2_server_event_loop]() { pi2_server_event_loop.Exit(); });
    pi2_server_event_loop.OnRun([quit, &pi2_server_event_loop]() {
      // Stop between timestamps, not exactly on them.
      quit->Setup(pi2_server_event_loop.monotonic_now() +
                  chrono::milliseconds(3050));
    });

    // And go!
    pi2_server_event_loop.Run();

    // And confirm we are synchronized again.
    EXPECT_TRUE(pi1_server_statistics_fetcher.Fetch());
    EXPECT_TRUE(pi2_server_statistics_fetcher.Fetch());

    const ServerConnection *const pi1_connection =
        pi1_server_statistics_fetcher->connections()->Get(0);
    const ServerConnection *const pi2_connection =
        pi2_server_statistics_fetcher->connections()->Get(0);

    EXPECT_EQ(pi1_connection->state(), State::CONNECTED);
    EXPECT_TRUE(pi1_connection->has_monotonic_offset());
    EXPECT_LT(chrono::nanoseconds(pi1_connection->monotonic_offset()),
              chrono::milliseconds(1));
    EXPECT_GT(chrono::nanoseconds(pi1_connection->monotonic_offset()),
              chrono::milliseconds(-1));
    EXPECT_TRUE(pi1_connection->has_boot_uuid());

    EXPECT_EQ(pi2_connection->state(), State::CONNECTED);
    EXPECT_TRUE(pi2_connection->has_monotonic_offset());
    EXPECT_LT(chrono::nanoseconds(pi2_connection->monotonic_offset()),
              chrono::milliseconds(1));
    EXPECT_GT(chrono::nanoseconds(pi2_connection->monotonic_offset()),
              chrono::milliseconds(-1));
    EXPECT_TRUE(pi2_connection->has_boot_uuid());
  }

  // Shut everyone else down
  pi1_server_event_loop.Exit();
  pi1_client_event_loop.Exit();
  pi2_client_event_loop.Exit();
  pi1_test_event_loop.Exit();
  pi2_test_event_loop.Exit();
  pi1_server_thread.join();
  pi1_client_thread.join();
  pi2_client_thread.join();
  pi1_test_thread.join();
  pi2_test_thread.join();
}

// TODO(austin): The above test confirms that the external state does the right
// thing, but doesn't confirm that the internal state does.  We either need to
// expose a way to check the state in a thread-safe way, or need a way to jump
// time for one node to do that.

void SendPing(aos::Sender<examples::Ping> *sender, int value) {
  aos::Sender<examples::Ping>::Builder builder = sender->MakeBuilder();
  examples::Ping::Builder ping_builder = builder.MakeBuilder<examples::Ping>();
  ping_builder.add_value(value);
  builder.Send(ping_builder.Finish());
}

// Tests that when a message is sent before the bridge starts up, but is
// configured as reliable, we forward it.  Confirm this survives a client reset.
TEST_F(MessageBridgeTest, ReliableSentBeforeClientStartup) {
  DoSetShmBase("pi1");
  // Force ourselves to be "raspberrypi" and allocate everything.
  FLAGS_override_hostname = "raspberrypi";

  FLAGS_application_name = "sender";
  aos::ShmEventLoop send_event_loop(&pi1_config.message());
  aos::Sender<examples::Ping> ping_sender =
      send_event_loop.MakeSender<examples::Ping>("/test");
  SendPing(&ping_sender, 1);
  aos::Sender<examples::Ping> unreliable_ping_sender =
      send_event_loop.MakeSender<examples::Ping>("/unreliable");
  SendPing(&unreliable_ping_sender, 1);

  FLAGS_application_name = "pi1_message_bridge_server";
  aos::ShmEventLoop pi1_server_event_loop(&pi1_config.message());
  MessageBridgeServer pi1_message_bridge_server(&pi1_server_event_loop);

  FLAGS_application_name = "pi1_message_bridge_client";
  aos::ShmEventLoop pi1_client_event_loop(&pi1_config.message());
  MessageBridgeClient pi1_message_bridge_client(&pi1_client_event_loop);

  FLAGS_application_name = "pi1_timestamp";
  aos::ShmEventLoop pi1_remote_timestamp_event_loop(&pi1_config.message());

  // Now do it for "raspberrypi2", the client.
  DoSetShmBase("pi2");
  FLAGS_override_hostname = "raspberrypi2";

  FLAGS_application_name = "pi2_message_bridge_server";
  aos::ShmEventLoop pi2_server_event_loop(&pi2_config.message());
  MessageBridgeServer pi2_message_bridge_server(&pi2_server_event_loop);

  aos::ShmEventLoop receive_event_loop(&pi2_config.message());
  aos::Fetcher<examples::Ping> ping_fetcher =
      receive_event_loop.MakeFetcher<examples::Ping>("/test");
  aos::Fetcher<examples::Ping> unreliable_ping_fetcher =
      receive_event_loop.MakeFetcher<examples::Ping>("/unreliable");
  aos::Fetcher<ClientStatistics> pi2_client_statistics_fetcher =
      receive_event_loop.MakeFetcher<ClientStatistics>("/pi2/aos");

  const size_t ping_channel_index = configuration::ChannelIndex(
      receive_event_loop.configuration(), ping_fetcher.channel());

  std::atomic<int> ping_timestamp_count{0};
  pi1_remote_timestamp_event_loop.MakeWatcher(
      "/pi1/aos/remote_timestamps/pi2",
      [ping_channel_index, &ping_timestamp_count](const RemoteMessage &header) {
        VLOG(1) << "/pi1/aos/remote_timestamps/pi2 RemoteMessage "
                << aos::FlatbufferToJson(&header);
        EXPECT_TRUE(header.has_boot_uuid());
        if (header.channel_index() == ping_channel_index) {
          ++ping_timestamp_count;
        }
      });

  // Before everything starts up, confirm there is no message.
  EXPECT_FALSE(ping_fetcher.Fetch());
  EXPECT_FALSE(unreliable_ping_fetcher.Fetch());

  // Spin up the persistant pieces.
  std::thread pi1_server_thread(
      [&pi1_server_event_loop]() { pi1_server_event_loop.Run(); });
  std::thread pi1_client_thread(
      [&pi1_client_event_loop]() { pi1_client_event_loop.Run(); });
  std::thread pi2_server_thread(
      [&pi2_server_event_loop]() { pi2_server_event_loop.Run(); });

  // Event used to wait for the timestamp counting thread to start.
  aos::Event event;
  std::thread pi1_remote_timestamp_thread(
      [&pi1_remote_timestamp_event_loop, &event]() {
        pi1_remote_timestamp_event_loop.OnRun([&event]() { event.Set(); });
        pi1_remote_timestamp_event_loop.Run();
      });

  event.Wait();

  {
    // Now, spin up a client for 2 seconds.
    LOG(INFO) << "Starting first pi2 MessageBridgeClient";
    FLAGS_application_name = "pi2_message_bridge_client";
    aos::ShmEventLoop pi2_client_event_loop(&pi2_config.message());
    MessageBridgeClient pi2_message_bridge_client(&pi2_client_event_loop);

    aos::TimerHandler *quit = pi2_client_event_loop.AddTimer(
        [&pi2_client_event_loop]() { pi2_client_event_loop.Exit(); });
    pi2_client_event_loop.OnRun([quit, &pi2_client_event_loop]() {
      // Stop between timestamps, not exactly on them.
      quit->Setup(pi2_client_event_loop.monotonic_now() +
                  chrono::milliseconds(2050));
    });

    // And go!
    pi2_client_event_loop.Run();

    // Confirm there is no detected duplicate packet.
    EXPECT_TRUE(pi2_client_statistics_fetcher.Fetch());
    EXPECT_EQ(pi2_client_statistics_fetcher->connections()
                  ->Get(0)
                  ->duplicate_packets(),
              0u);

    EXPECT_TRUE(ping_fetcher.Fetch());
    EXPECT_FALSE(unreliable_ping_fetcher.Fetch());
    EXPECT_EQ(ping_timestamp_count, 1);
    LOG(INFO) << "Shutting down first pi2 MessageBridgeClient";
  }

  {
    // Now, spin up a second client for 2 seconds.
    LOG(INFO) << "Starting second pi2 MessageBridgeClient";
    FLAGS_application_name = "pi2_message_bridge_client";
    aos::ShmEventLoop pi2_client_event_loop(&pi2_config.message());
    MessageBridgeClient pi2_message_bridge_client(&pi2_client_event_loop);

    aos::TimerHandler *quit = pi2_client_event_loop.AddTimer(
        [&pi2_client_event_loop]() { pi2_client_event_loop.Exit(); });
    pi2_client_event_loop.OnRun([quit, &pi2_client_event_loop]() {
      // Stop between timestamps, not exactly on them.
      quit->Setup(pi2_client_event_loop.monotonic_now() +
                  chrono::milliseconds(5050));
    });

    // And go!
    pi2_client_event_loop.Run();

    // Confirm we detect the duplicate packet correctly.
    EXPECT_TRUE(pi2_client_statistics_fetcher.Fetch());
    EXPECT_EQ(pi2_client_statistics_fetcher->connections()
                  ->Get(0)
                  ->duplicate_packets(),
              1u);

    EXPECT_EQ(ping_timestamp_count, 1);
    EXPECT_FALSE(ping_fetcher.Fetch());
    EXPECT_FALSE(unreliable_ping_fetcher.Fetch());
  }

  // Shut everyone else down
  pi1_server_event_loop.Exit();
  pi1_client_event_loop.Exit();
  pi2_server_event_loop.Exit();
  pi1_remote_timestamp_event_loop.Exit();
  pi1_remote_timestamp_thread.join();
  pi1_server_thread.join();
  pi1_client_thread.join();
  pi2_server_thread.join();
}

// Tests that when a message is sent before the bridge starts up, but is
// configured as reliable, we forward it.  Confirm this works across server
// resets.
TEST_F(MessageBridgeTest, ReliableSentBeforeServerStartup) {
  // Now do it for "raspberrypi2", the client.
  DoSetShmBase("pi2");
  FLAGS_override_hostname = "raspberrypi2";

  FLAGS_application_name = "pi2_message_bridge_server";
  aos::ShmEventLoop pi2_server_event_loop(&pi2_config.message());
  MessageBridgeServer pi2_message_bridge_server(&pi2_server_event_loop);

  FLAGS_application_name = "pi2_message_bridge_client";
  aos::ShmEventLoop pi2_client_event_loop(&pi2_config.message());
  MessageBridgeClient pi2_message_bridge_client(&pi2_client_event_loop);

  aos::ShmEventLoop receive_event_loop(&pi2_config.message());
  aos::Fetcher<examples::Ping> ping_fetcher =
      receive_event_loop.MakeFetcher<examples::Ping>("/test");
  aos::Fetcher<examples::Ping> unreliable_ping_fetcher =
      receive_event_loop.MakeFetcher<examples::Ping>("/unreliable");
  aos::Fetcher<ClientStatistics> pi2_client_statistics_fetcher =
      receive_event_loop.MakeFetcher<ClientStatistics>("/pi2/aos");

  DoSetShmBase("pi1");
  // Force ourselves to be "raspberrypi" and allocate everything.
  FLAGS_override_hostname = "raspberrypi";

  FLAGS_application_name = "sender";
  aos::ShmEventLoop send_event_loop(&pi1_config.message());
  aos::Sender<examples::Ping> ping_sender =
      send_event_loop.MakeSender<examples::Ping>("/test");
  {
    aos::Sender<examples::Ping>::Builder builder = ping_sender.MakeBuilder();
    examples::Ping::Builder ping_builder =
        builder.MakeBuilder<examples::Ping>();
    ping_builder.add_value(1);
    builder.Send(ping_builder.Finish());
  }

  FLAGS_application_name = "pi1_message_bridge_client";
  aos::ShmEventLoop pi1_client_event_loop(&pi1_config.message());
  MessageBridgeClient pi1_message_bridge_client(&pi1_client_event_loop);

  FLAGS_application_name = "pi1_timestamp";
  aos::ShmEventLoop pi1_remote_timestamp_event_loop(&pi1_config.message());

  const size_t ping_channel_index = configuration::ChannelIndex(
      receive_event_loop.configuration(), ping_fetcher.channel());

  std::atomic<int> ping_timestamp_count{0};
  pi1_remote_timestamp_event_loop.MakeWatcher(
      "/pi1/aos/remote_timestamps/pi2",
      [ping_channel_index, &ping_timestamp_count](const RemoteMessage &header) {
        VLOG(1) << "/pi1/aos/remote_timestamps/pi2 RemoteMessage "
                << aos::FlatbufferToJson(&header);
        EXPECT_TRUE(header.has_boot_uuid());
        if (header.channel_index() == ping_channel_index) {
          ++ping_timestamp_count;
        }
      });

  // Before everything starts up, confirm there is no message.
  EXPECT_FALSE(ping_fetcher.Fetch());
  EXPECT_FALSE(unreliable_ping_fetcher.Fetch());

  // Spin up the persistant pieces.
  std::thread pi1_client_thread(
      [&pi1_client_event_loop]() { pi1_client_event_loop.Run(); });
  std::thread pi2_server_thread(
      [&pi2_server_event_loop]() { pi2_server_event_loop.Run(); });
  std::thread pi2_client_thread(
      [&pi2_client_event_loop]() { pi2_client_event_loop.Run(); });

  // Event used to wait for the timestamp counting thread to start.
  aos::Event event;
  std::thread pi1_remote_timestamp_thread(
      [&pi1_remote_timestamp_event_loop, &event]() {
        pi1_remote_timestamp_event_loop.OnRun([&event]() { event.Set(); });
        pi1_remote_timestamp_event_loop.Run();
      });

  event.Wait();

  {
    // Now, spin up a server for 2 seconds.
    FLAGS_application_name = "pi1_message_bridge_server";
    aos::ShmEventLoop pi1_server_event_loop(&pi1_config.message());
    MessageBridgeServer pi1_message_bridge_server(&pi1_server_event_loop);

    aos::TimerHandler *quit = pi1_server_event_loop.AddTimer(
        [&pi1_server_event_loop]() { pi1_server_event_loop.Exit(); });
    pi1_server_event_loop.OnRun([quit, &pi1_server_event_loop]() {
      // Stop between timestamps, not exactly on them.
      quit->Setup(pi1_server_event_loop.monotonic_now() +
                  chrono::milliseconds(2050));
    });

    // And go!
    pi1_server_event_loop.Run();

    // Confirm there is no detected duplicate packet.
    EXPECT_TRUE(pi2_client_statistics_fetcher.Fetch());
    EXPECT_EQ(pi2_client_statistics_fetcher->connections()
                  ->Get(0)
                  ->duplicate_packets(),
              0u);

    EXPECT_TRUE(ping_fetcher.Fetch());
    EXPECT_FALSE(unreliable_ping_fetcher.Fetch());
    EXPECT_EQ(ping_timestamp_count, 1);
    LOG(INFO) << "Shutting down first pi1 MessageBridgeServer";
  }

  {
    // Now, spin up a second server for 2 seconds.
    FLAGS_application_name = "pi1_message_bridge_server";
    aos::ShmEventLoop pi1_server_event_loop(&pi1_config.message());
    MessageBridgeServer pi1_message_bridge_server(&pi1_server_event_loop);

    aos::TimerHandler *quit = pi1_server_event_loop.AddTimer(
        [&pi1_server_event_loop]() { pi1_server_event_loop.Exit(); });
    pi1_server_event_loop.OnRun([quit, &pi1_server_event_loop]() {
      // Stop between timestamps, not exactly on them.
      quit->Setup(pi1_server_event_loop.monotonic_now() +
                  chrono::milliseconds(2050));
    });

    // And go!
    pi1_server_event_loop.Run();

    // Confirm we detect the duplicate packet correctly.
    EXPECT_TRUE(pi2_client_statistics_fetcher.Fetch());
    EXPECT_EQ(pi2_client_statistics_fetcher->connections()
                  ->Get(0)
                  ->duplicate_packets(),
              1u);

    EXPECT_EQ(ping_timestamp_count, 1);
    EXPECT_FALSE(ping_fetcher.Fetch());
    EXPECT_FALSE(unreliable_ping_fetcher.Fetch());
    LOG(INFO) << "Shutting down first pi1 MessageBridgeServer";
  }

  // Shut everyone else down
  pi1_client_event_loop.Exit();
  pi2_server_event_loop.Exit();
  pi2_client_event_loop.Exit();
  pi1_remote_timestamp_event_loop.Exit();
  pi1_remote_timestamp_thread.join();
  pi1_client_thread.join();
  pi2_server_thread.join();
  pi2_client_thread.join();
}

}  // namespace testing
}  // namespace message_bridge
}  // namespace aos
