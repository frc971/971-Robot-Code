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

namespace aos {

namespace message_bridge {
namespace testing {

// Note: All of these tests spin up ShmEventLoop's in separate threads to allow
// us to run the "real" message bridge. This requires extra threading and timing
// coordination to make happen, which is the reason for some of the extra
// complexity in these tests.

// Test that we can send a ping message over sctp and receive it.
TEST_P(MessageBridgeParameterizedTest, PingPong) {
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
  OnPi1();
  // Force ourselves to be "raspberrypi" and allocate everything.

  MakePi1Server();
  MakePi1Client();

  const std::string long_data = std::string(10000, 'a');

  // And build the app which sends the pings.
  FLAGS_application_name = "ping";
  aos::ShmEventLoop ping_event_loop(&config.message());
  aos::Sender<examples::Ping> ping_sender =
      ping_event_loop.MakeSender<examples::Ping>("/test");

  aos::ShmEventLoop pi1_test_event_loop(&config.message());
  aos::Fetcher<RemoteMessage> message_header_fetcher1 =
      pi1_test_event_loop.MakeFetcher<RemoteMessage>(
          shared() ? "/pi1/aos/remote_timestamps/pi2"
                   : "/pi1/aos/remote_timestamps/pi2/test/aos-examples-Ping");

  // Fetchers for confirming the remote timestamps made it.
  aos::Fetcher<examples::Ping> ping_on_pi1_fetcher =
      ping_event_loop.MakeFetcher<examples::Ping>("/test");
  aos::Fetcher<Timestamp> pi1_on_pi1_timestamp_fetcher =
      ping_event_loop.MakeFetcher<Timestamp>("/aos");

  // Now do it for "raspberrypi2", the client.
  OnPi2();

  MakePi2Client();
  MakePi2Server();

  // And build the app which sends the pongs.
  FLAGS_application_name = "pong";
  aos::ShmEventLoop pong_event_loop(&config.message());

  // And build the app for testing.
  FLAGS_application_name = "test";
  aos::ShmEventLoop test_event_loop(&config.message());

  aos::Fetcher<ClientStatistics> client_statistics_fetcher =
      test_event_loop.MakeFetcher<ClientStatistics>("/aos");
  aos::Fetcher<RemoteMessage> message_header_fetcher2 =
      test_event_loop.MakeFetcher<RemoteMessage>(
          shared() ? "/pi2/aos/remote_timestamps/pi1"
                   : "/pi2/aos/remote_timestamps/pi1/pi2/aos/"
                     "aos-message_bridge-Timestamp");

  // Event loop for fetching data delivered to pi2 from pi1 to match up
  // messages.
  aos::ShmEventLoop delivered_messages_event_loop(&config.message());
  aos::Fetcher<Timestamp> pi1_on_pi2_timestamp_fetcher =
      delivered_messages_event_loop.MakeFetcher<Timestamp>("/pi1/aos");
  aos::Fetcher<examples::Ping> ping_on_pi2_fetcher =
      delivered_messages_event_loop.MakeFetcher<examples::Ping>("/test");
  EXPECT_FALSE(ping_on_pi2_fetcher.Fetch());
  EXPECT_FALSE(pi1_on_pi2_timestamp_fetcher.Fetch());

  // Count the pongs.
  int pong_count = 0;
  pong_event_loop.MakeWatcher("/test", [&pong_count, &pong_event_loop,
                                        this](const examples::Ping &ping) {
    EXPECT_EQ(pong_event_loop.context().source_boot_uuid, pi1_boot_uuid_);
    ++pong_count;
    VLOG(1) << "Got ping back " << FlatbufferToJson(&ping);
  });

  FLAGS_override_hostname = "";

  // Wait until we are connected, then send.
  int ping_count = 0;
  int pi1_server_statistics_count = 0;
  ping_event_loop.MakeWatcher(
      "/pi1/aos",
      [this, &ping_count, &ping_sender, &pi1_server_statistics_count,
       &long_data](const ServerStatistics &stats) {
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
              pi2_client_event_loop->node()->name()->string_view()) {
            if (connection->state() == State::CONNECTED) {
              EXPECT_TRUE(connection->has_boot_uuid());
              EXPECT_EQ(connection->connection_count(), 1u);
              EXPECT_LT(monotonic_clock::time_point(chrono::nanoseconds(
                            connection->connected_since_time())),
                        monotonic_clock::now());
              connected = true;
            } else {
              EXPECT_FALSE(connection->has_connection_count());
              EXPECT_FALSE(connection->has_connected_since_time());
            }
          }
        }

        if (connected) {
          VLOG(1) << "Connected!  Sent ping.";
          auto builder = ping_sender.MakeBuilder();
          builder.fbb()->CreateString(long_data);
          examples::Ping::Builder ping_builder =
              builder.MakeBuilder<examples::Ping>();
          ping_builder.add_value(ping_count + 971);
          EXPECT_EQ(builder.Send(ping_builder.Finish()), RawSender::Error::kOk);
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

      if (connection->state() == State::CONNECTED) {
        EXPECT_EQ(connection->connection_count(), 1u);
        EXPECT_LT(monotonic_clock::time_point(
                      chrono::nanoseconds(connection->connected_since_time())),
                  monotonic_clock::now());
      } else {
        // If we have been connected, we expect the connection count to stay
        // around.
        if (pi2_server_statistics_count > 0) {
          EXPECT_TRUE(connection->has_connection_count());
          EXPECT_EQ(connection->connection_count(), 1u);
        } else {
          EXPECT_FALSE(connection->has_connection_count());
        }
        EXPECT_FALSE(connection->has_connected_since_time());
      }
    }
  });

  int pi1_client_statistics_count = 0;
  int pi1_connected_client_statistics_count = 0;
  ping_event_loop.MakeWatcher(
      "/pi1/aos",
      [&pi1_client_statistics_count,
       &pi1_connected_client_statistics_count](const ClientStatistics &stats) {
        VLOG(1) << "/pi1/aos ClientStatistics " << FlatbufferToJson(&stats);

        for (const ClientConnection *connection : *stats.connections()) {
          if (connection->has_monotonic_offset()) {
            ++pi1_client_statistics_count;
            // It takes at least 10 microseconds to send a message between the
            // client and server.  The min (filtered) time shouldn't be over 10
            // milliseconds on localhost.  This might have to bump up if this is
            // proving flaky.
            EXPECT_LT(chrono::nanoseconds(connection->monotonic_offset()),
                      chrono::milliseconds(10))
                << " " << connection->monotonic_offset()
                << "ns vs 10000ns on iteration " << pi1_client_statistics_count;
            EXPECT_GT(chrono::nanoseconds(connection->monotonic_offset()),
                      chrono::microseconds(10))
                << " " << connection->monotonic_offset()
                << "ns vs 10000ns on iteration " << pi1_client_statistics_count;
          }
          if (connection->state() == State::CONNECTED) {
            EXPECT_EQ(connection->connection_count(), 1u);
            EXPECT_LT(monotonic_clock::time_point(chrono::nanoseconds(
                          connection->connected_since_time())),
                      monotonic_clock::now());
            // The first Connected message may not have a UUID in it since no
            // data has flown.  That's fine.
            if (pi1_connected_client_statistics_count > 0) {
              EXPECT_TRUE(connection->has_boot_uuid())
                  << ": " << aos::FlatbufferToJson(connection);
            }
            ++pi1_connected_client_statistics_count;
          } else {
            EXPECT_FALSE(connection->has_connection_count());
            EXPECT_FALSE(connection->has_connected_since_time());
          }
        }
      });

  int pi2_client_statistics_count = 0;
  int pi2_connected_client_statistics_count = 0;
  pong_event_loop.MakeWatcher(
      "/pi2/aos",
      [&pi2_client_statistics_count,
       &pi2_connected_client_statistics_count](const ClientStatistics &stats) {
        VLOG(1) << "/pi2/aos ClientStatistics " << FlatbufferToJson(&stats);

        for (const ClientConnection *connection : *stats.connections()) {
          if (connection->has_monotonic_offset()) {
            ++pi2_client_statistics_count;
            EXPECT_LT(chrono::nanoseconds(connection->monotonic_offset()),
                      chrono::milliseconds(10))
                << ": got " << aos::FlatbufferToJson(connection);
            EXPECT_GT(chrono::nanoseconds(connection->monotonic_offset()),
                      chrono::microseconds(10))
                << ": got " << aos::FlatbufferToJson(connection);
          }
          if (connection->state() == State::CONNECTED) {
            EXPECT_EQ(connection->connection_count(), 1u);
            EXPECT_LT(monotonic_clock::time_point(chrono::nanoseconds(
                          connection->connected_since_time())),
                      monotonic_clock::now());
            if (pi2_connected_client_statistics_count > 0) {
              EXPECT_TRUE(connection->has_boot_uuid());
            }
            ++pi2_connected_client_statistics_count;
          } else {
            if (pi2_connected_client_statistics_count == 0) {
              EXPECT_FALSE(connection->has_connection_count())
                  << aos::FlatbufferToJson(&stats);
            } else {
              EXPECT_TRUE(connection->has_connection_count())
                  << aos::FlatbufferToJson(&stats);
              EXPECT_EQ(connection->connection_count(), 1u);
            }
            EXPECT_FALSE(connection->has_connected_since_time());
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
  for (std::pair<int, std::string> channel :
       shared()
           ? std::vector<std::pair<
                 int, std::string>>{{-1, "/pi1/aos/remote_timestamps/pi2"}}
           : std::vector<std::pair<int, std::string>>{
                 {pi1_timestamp_channel,
                  "/pi1/aos/remote_timestamps/pi2/pi1/aos/"
                  "aos-message_bridge-Timestamp"},
                 {ping_timestamp_channel,
                  "/pi1/aos/remote_timestamps/pi2/test/aos-examples-Ping"}}) {
    ping_event_loop.MakeWatcher(
        channel.second,
        [pi1_timestamp_channel, ping_timestamp_channel, &ping_on_pi2_fetcher,
         &ping_on_pi1_fetcher, &pi1_on_pi2_timestamp_fetcher,
         &pi1_on_pi1_timestamp_fetcher,
         channel_index = channel.first](const RemoteMessage &header) {
          VLOG(1) << "/pi1/aos/remote_timestamps/pi2 RemoteMessage "
                  << aos::FlatbufferToJson(&header);

          EXPECT_TRUE(header.has_boot_uuid());
          if (channel_index != -1) {
            ASSERT_EQ(channel_index, header.channel_index());
          }

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
          EXPECT_EQ(pi2_context->realtime_event_time,
                    header_realtime_sent_time);
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
  }

  // Start everything up.  Pong is the only thing we don't know how to wait
  // on, so start it first.
  ThreadedEventLoopRunner pong_thread(&pong_event_loop);
  ThreadedEventLoopRunner ping_thread(&ping_event_loop);

  StartPi1Server();
  StartPi1Client();
  StartPi2Client();
  StartPi2Server();

  // And go!
  // Run for 5 seconds to make sure we have time to estimate the offset.
  std::this_thread::sleep_for(chrono::milliseconds(5050));

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
      client_statistics_fetcher->connections()->Get(0)->monotonic_offset(), 0)
      << aos::FlatbufferToJson(client_statistics_fetcher.get());
  EXPECT_LT(
      client_statistics_fetcher->connections()->Get(0)->monotonic_offset(),
      1000000000)
      << aos::FlatbufferToJson(client_statistics_fetcher.get());

  // Shut everyone else down before confirming everything actually ran.
  ping_thread.Exit();
  pong_thread.Exit();
  StopPi1Server();
  StopPi1Client();
  StopPi2Client();
  StopPi2Server();

  // Make sure we sent something.
  EXPECT_GE(ping_count, 1);
  // And got something back.
  EXPECT_GE(pong_count, 1);

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
TEST_P(MessageBridgeParameterizedTest, ClientRestart) {
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
  OnPi1();

  MakePi1Server();
  MakePi1Client();

  // And build the app for testing.
  MakePi1Test();
  aos::Fetcher<ServerStatistics> pi1_server_statistics_fetcher =
      pi1_test_event_loop->MakeFetcher<ServerStatistics>("/pi1/aos");

  // Now do it for "raspberrypi2", the client.
  OnPi2();
  MakePi2Server();

  // And build the app for testing.
  MakePi2Test();
  aos::Fetcher<ServerStatistics> pi2_server_statistics_fetcher =
      pi2_test_event_loop->MakeFetcher<ServerStatistics>("/pi2/aos");

  // Wait until we are connected, then send.

  StartPi1Test();
  StartPi2Test();
  StartPi1Server();
  StartPi1Client();
  StartPi2Server();

  {
    MakePi2Client();

    RunPi2Client(chrono::milliseconds(3050));

    // Now confirm we are synchronized.
    EXPECT_TRUE(pi1_server_statistics_fetcher.Fetch());
    EXPECT_TRUE(pi2_server_statistics_fetcher.Fetch());

    const ServerConnection *const pi1_connection =
        pi1_server_statistics_fetcher->connections()->Get(0);
    const ServerConnection *const pi2_connection =
        pi2_server_statistics_fetcher->connections()->Get(0);

    EXPECT_EQ(pi1_connection->state(), State::CONNECTED);
    EXPECT_EQ(pi1_connection->connection_count(), 1u);
    EXPECT_TRUE(pi1_connection->has_connected_since_time());
    EXPECT_TRUE(pi1_connection->has_monotonic_offset());
    EXPECT_LT(chrono::nanoseconds(pi1_connection->monotonic_offset()),
              chrono::milliseconds(1));
    EXPECT_GT(chrono::nanoseconds(pi1_connection->monotonic_offset()),
              chrono::milliseconds(-1));
    EXPECT_TRUE(pi1_connection->has_boot_uuid());

    EXPECT_EQ(pi2_connection->state(), State::CONNECTED);
    EXPECT_EQ(pi2_connection->connection_count(), 1u);
    EXPECT_TRUE(pi2_connection->has_connected_since_time());
    EXPECT_TRUE(pi2_connection->has_monotonic_offset());
    EXPECT_LT(chrono::nanoseconds(pi2_connection->monotonic_offset()),
              chrono::milliseconds(1));
    EXPECT_GT(chrono::nanoseconds(pi2_connection->monotonic_offset()),
              chrono::milliseconds(-1));
    EXPECT_TRUE(pi2_connection->has_boot_uuid());

    StopPi2Client();
  }

  std::this_thread::sleep_for(SctpClientConnection::kReconnectTimeout +
                              std::chrono::seconds(1));

  {
    // Now confirm we are un-synchronized.
    EXPECT_TRUE(pi1_server_statistics_fetcher.Fetch());
    EXPECT_TRUE(pi2_server_statistics_fetcher.Fetch());
    const ServerConnection *const pi1_connection =
        pi1_server_statistics_fetcher->connections()->Get(0);
    const ServerConnection *const pi2_connection =
        pi2_server_statistics_fetcher->connections()->Get(0);

    EXPECT_EQ(pi1_connection->state(), State::DISCONNECTED);
    EXPECT_EQ(pi1_connection->connection_count(), 1u);
    EXPECT_FALSE(pi1_connection->has_connected_since_time());
    EXPECT_FALSE(pi1_connection->has_monotonic_offset());
    EXPECT_FALSE(pi1_connection->has_boot_uuid());
    EXPECT_EQ(pi2_connection->state(), State::CONNECTED);
    EXPECT_FALSE(pi2_connection->has_monotonic_offset());
    EXPECT_TRUE(pi2_connection->has_boot_uuid());
    EXPECT_EQ(pi2_connection->connection_count(), 1u);
    EXPECT_TRUE(pi2_connection->has_connected_since_time());
  }

  {
    MakePi2Client();
    // And go!
    RunPi2Client(chrono::milliseconds(3050));

    EXPECT_TRUE(pi1_server_statistics_fetcher.Fetch());
    EXPECT_TRUE(pi2_server_statistics_fetcher.Fetch());

    // Now confirm we are synchronized again.
    const ServerConnection *const pi1_connection =
        pi1_server_statistics_fetcher->connections()->Get(0);
    const ServerConnection *const pi2_connection =
        pi2_server_statistics_fetcher->connections()->Get(0);

    EXPECT_EQ(pi1_connection->state(), State::CONNECTED);
    EXPECT_EQ(pi1_connection->connection_count(), 2u);
    EXPECT_TRUE(pi1_connection->has_connected_since_time());
    EXPECT_TRUE(pi1_connection->has_monotonic_offset());
    EXPECT_LT(chrono::nanoseconds(pi1_connection->monotonic_offset()),
              chrono::milliseconds(1))
        << ": " << FlatbufferToJson(pi1_connection);
    EXPECT_GT(chrono::nanoseconds(pi1_connection->monotonic_offset()),
              chrono::milliseconds(-1))
        << ": " << FlatbufferToJson(pi1_connection);
    EXPECT_TRUE(pi1_connection->has_boot_uuid());

    EXPECT_EQ(pi2_connection->state(), State::CONNECTED);
    EXPECT_EQ(pi2_connection->connection_count(), 1u);
    EXPECT_TRUE(pi2_connection->has_connected_since_time());
    EXPECT_TRUE(pi2_connection->has_monotonic_offset());
    EXPECT_LT(chrono::nanoseconds(pi2_connection->monotonic_offset()),
              chrono::milliseconds(1))
        << ": " << FlatbufferToJson(pi2_connection);
    EXPECT_GT(chrono::nanoseconds(pi2_connection->monotonic_offset()),
              chrono::milliseconds(-1))
        << ": " << FlatbufferToJson(pi2_connection);
    EXPECT_TRUE(pi2_connection->has_boot_uuid());

    StopPi2Client();
  }

  // Shut everyone else down.
  StopPi1Server();
  StopPi1Client();
  StopPi2Server();
  StopPi1Test();
  StopPi2Test();
}

// Test that the server disconnecting triggers the server offsets on the other
// side to clear, along with the other client.
TEST_P(MessageBridgeParameterizedTest, ServerRestart) {
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
  // Force ourselves to be "raspberrypi" and allocate everything.
  OnPi1();
  MakePi1Server();
  MakePi1Client();

  // And build the app for testing.
  MakePi1Test();
  aos::Fetcher<ServerStatistics> pi1_server_statistics_fetcher =
      pi1_test_event_loop->MakeFetcher<ServerStatistics>("/pi1/aos");
  aos::Fetcher<ClientStatistics> pi1_client_statistics_fetcher =
      pi1_test_event_loop->MakeFetcher<ClientStatistics>("/pi1/aos");

  // Now do it for "raspberrypi2", the client.
  OnPi2();
  MakePi2Client();

  // And build the app for testing.
  MakePi2Test();
  aos::Fetcher<ServerStatistics> pi2_server_statistics_fetcher =
      pi2_test_event_loop->MakeFetcher<ServerStatistics>("/pi2/aos");

  // Start everything up.  Pong is the only thing we don't know how to wait on,
  // so start it first.
  StartPi1Test();
  StartPi2Test();
  StartPi1Server();
  StartPi1Client();
  StartPi2Client();

  // Confirm both client and server statistics messages have decent offsets in
  // them.

  {
    MakePi2Server();

    RunPi2Server(chrono::milliseconds(3050));

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
    EXPECT_TRUE(pi1_connection->has_connected_since_time());
    EXPECT_EQ(pi1_connection->connection_count(), 1u);

    EXPECT_EQ(pi2_connection->state(), State::CONNECTED);
    EXPECT_TRUE(pi2_connection->has_monotonic_offset());
    EXPECT_LT(chrono::nanoseconds(pi2_connection->monotonic_offset()),
              chrono::milliseconds(1));
    EXPECT_GT(chrono::nanoseconds(pi2_connection->monotonic_offset()),
              chrono::milliseconds(-1));
    EXPECT_TRUE(pi2_connection->has_boot_uuid());
    EXPECT_TRUE(pi2_connection->has_connected_since_time());
    EXPECT_EQ(pi2_connection->connection_count(), 1u);

    StopPi2Server();
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
    EXPECT_TRUE(pi1_server_connection->has_connected_since_time());
    EXPECT_EQ(pi1_server_connection->connection_count(), 1u);

    EXPECT_TRUE(pi1_server_connection->has_boot_uuid());
    EXPECT_EQ(pi1_client_connection->state(), State::DISCONNECTED);
    EXPECT_FALSE(pi1_client_connection->has_monotonic_offset());
    EXPECT_FALSE(pi1_client_connection->has_connected_since_time());
    EXPECT_EQ(pi1_client_connection->connection_count(), 1u);
    EXPECT_FALSE(pi1_client_connection->has_boot_uuid());
  }

  {
    MakePi2Server();

    RunPi2Server(chrono::milliseconds(3050));

    // And confirm we are synchronized again.
    EXPECT_TRUE(pi1_server_statistics_fetcher.Fetch());
    EXPECT_TRUE(pi2_server_statistics_fetcher.Fetch());
    EXPECT_TRUE(pi1_client_statistics_fetcher.Fetch());

    const ServerConnection *const pi1_connection =
        pi1_server_statistics_fetcher->connections()->Get(0);
    const ServerConnection *const pi2_connection =
        pi2_server_statistics_fetcher->connections()->Get(0);
    const ClientConnection *const pi1_client_connection =
        pi1_client_statistics_fetcher->connections()->Get(0);

    EXPECT_EQ(pi1_connection->state(), State::CONNECTED);
    EXPECT_TRUE(pi1_connection->has_monotonic_offset());
    EXPECT_LT(chrono::nanoseconds(pi1_connection->monotonic_offset()),
              chrono::milliseconds(1));
    EXPECT_GT(chrono::nanoseconds(pi1_connection->monotonic_offset()),
              chrono::milliseconds(-1));
    EXPECT_TRUE(pi1_connection->has_boot_uuid());

    EXPECT_EQ(pi1_client_connection->state(), State::CONNECTED);
    EXPECT_TRUE(pi1_client_connection->has_connected_since_time());
    EXPECT_EQ(pi1_client_connection->connection_count(), 2u);
    EXPECT_TRUE(pi1_client_connection->has_boot_uuid());

    EXPECT_EQ(pi2_connection->state(), State::CONNECTED);
    EXPECT_TRUE(pi2_connection->has_monotonic_offset());
    EXPECT_LT(chrono::nanoseconds(pi2_connection->monotonic_offset()),
              chrono::milliseconds(1));
    EXPECT_GT(chrono::nanoseconds(pi2_connection->monotonic_offset()),
              chrono::milliseconds(-1));
    EXPECT_TRUE(pi2_connection->has_boot_uuid());

    StopPi2Server();
  }

  // Shut everyone else down.
  StopPi1Server();
  StopPi1Client();
  StopPi2Client();
  StopPi1Test();
  StopPi2Test();
}

// TODO(austin): The above test confirms that the external state does the right
// thing, but doesn't confirm that the internal state does.  We either need to
// expose a way to check the state in a thread-safe way, or need a way to jump
// time for one node to do that.

void SendPing(aos::Sender<examples::Ping> *sender, int value) {
  aos::Sender<examples::Ping>::Builder builder = sender->MakeBuilder();
  examples::Ping::Builder ping_builder = builder.MakeBuilder<examples::Ping>();
  ping_builder.add_value(value);
  builder.CheckOk(builder.Send(ping_builder.Finish()));
}

// Tests that when a message is sent before the bridge starts up, but is
// configured as reliable, we forward it.  Confirm this survives a client reset.
TEST_P(MessageBridgeParameterizedTest, ReliableSentBeforeClientStartup) {
  OnPi1();

  FLAGS_application_name = "sender";
  aos::ShmEventLoop send_event_loop(&config.message());
  aos::Sender<examples::Ping> ping_sender =
      send_event_loop.MakeSender<examples::Ping>("/test");
  SendPing(&ping_sender, 1);
  aos::Sender<examples::Ping> unreliable_ping_sender =
      send_event_loop.MakeSender<examples::Ping>("/unreliable");
  SendPing(&unreliable_ping_sender, 1);

  MakePi1Server();
  MakePi1Client();

  FLAGS_application_name = "pi1_timestamp";
  aos::ShmEventLoop pi1_remote_timestamp_event_loop(&config.message());

  // Now do it for "raspberrypi2", the client.
  OnPi2();

  MakePi2Server();

  aos::ShmEventLoop receive_event_loop(&config.message());
  aos::Fetcher<examples::Ping> ping_fetcher =
      receive_event_loop.MakeFetcher<examples::Ping>("/test");
  aos::Fetcher<examples::Ping> unreliable_ping_fetcher =
      receive_event_loop.MakeFetcher<examples::Ping>("/unreliable");
  aos::Fetcher<ClientStatistics> pi2_client_statistics_fetcher =
      receive_event_loop.MakeFetcher<ClientStatistics>("/pi2/aos");

  const size_t ping_channel_index = configuration::ChannelIndex(
      receive_event_loop.configuration(), ping_fetcher.channel());

  // ping_timestamp_count is accessed from multiple threads (the Watcher that
  // triggers it is in a separate thread), so make it atomic.
  std::atomic<int> ping_timestamp_count{0};
  const std::string channel_name =
      shared() ? "/pi1/aos/remote_timestamps/pi2"
               : "/pi1/aos/remote_timestamps/pi2/test/aos-examples-Ping";
  pi1_remote_timestamp_event_loop.MakeWatcher(
      channel_name, [this, channel_name, ping_channel_index,
                     &ping_timestamp_count](const RemoteMessage &header) {
        VLOG(1) << channel_name << " RemoteMessage "
                << aos::FlatbufferToJson(&header);
        EXPECT_TRUE(header.has_boot_uuid());
        if (shared() && header.channel_index() != ping_channel_index) {
          return;
        }
        CHECK_EQ(header.channel_index(), ping_channel_index);
        ++ping_timestamp_count;
      });

  // Before everything starts up, confirm there is no message.
  EXPECT_FALSE(ping_fetcher.Fetch());
  EXPECT_FALSE(unreliable_ping_fetcher.Fetch());

  // Spin up the persistent pieces.
  StartPi1Server();
  StartPi1Client();
  StartPi2Server();

  // Event used to wait for the timestamp counting thread to start.
  std::unique_ptr<ThreadedEventLoopRunner> pi1_remote_timestamp_thread =
      std::make_unique<ThreadedEventLoopRunner>(
          &pi1_remote_timestamp_event_loop);

  {
    // Now spin up a client for 2 seconds.
    MakePi2Client();

    RunPi2Client(chrono::milliseconds(2050));

    // Confirm there is no detected duplicate packet.
    EXPECT_TRUE(pi2_client_statistics_fetcher.Fetch());
    EXPECT_EQ(pi2_client_statistics_fetcher->connections()
                  ->Get(0)
                  ->duplicate_packets(),
              0u);

    EXPECT_EQ(pi2_client_statistics_fetcher->connections()
                  ->Get(0)
                  ->partial_deliveries(),
              0u);

    EXPECT_TRUE(ping_fetcher.Fetch());
    EXPECT_FALSE(unreliable_ping_fetcher.Fetch());
    EXPECT_EQ(ping_timestamp_count, 1);

    StopPi2Client();
  }

  {
    // Now, spin up a client for 2 seconds.
    MakePi2Client();

    RunPi2Client(chrono::milliseconds(5050));

    // Confirm we detect the duplicate packet correctly.
    EXPECT_TRUE(pi2_client_statistics_fetcher.Fetch());
    EXPECT_EQ(pi2_client_statistics_fetcher->connections()
                  ->Get(0)
                  ->duplicate_packets(),
              1u);

    EXPECT_EQ(pi2_client_statistics_fetcher->connections()
                  ->Get(0)
                  ->partial_deliveries(),
              0u);

    EXPECT_EQ(ping_timestamp_count, 1);
    EXPECT_FALSE(ping_fetcher.Fetch());
    EXPECT_FALSE(unreliable_ping_fetcher.Fetch());

    StopPi2Client();
  }

  // Shut everyone else down.
  StopPi1Client();
  StopPi2Server();
  pi1_remote_timestamp_thread.reset();
  StopPi1Server();
}

// Tests that when a message is sent before the bridge starts up, but is
// configured as reliable, we forward it.  Confirm this works across server
// resets.
TEST_P(MessageBridgeParameterizedTest, ReliableSentBeforeServerStartup) {
  // Now do it for "raspberrypi2", the client.
  OnPi2();

  MakePi2Server();
  MakePi2Client();

  aos::ShmEventLoop receive_event_loop(&config.message());
  aos::Fetcher<examples::Ping> ping_fetcher =
      receive_event_loop.MakeFetcher<examples::Ping>("/test");
  aos::Fetcher<examples::Ping> unreliable_ping_fetcher =
      receive_event_loop.MakeFetcher<examples::Ping>("/unreliable");
  aos::Fetcher<ClientStatistics> pi2_client_statistics_fetcher =
      receive_event_loop.MakeFetcher<ClientStatistics>("/pi2/aos");

  // Force ourselves to be "raspberrypi" and allocate everything.
  OnPi1();

  FLAGS_application_name = "sender";
  aos::ShmEventLoop send_event_loop(&config.message());
  aos::Sender<examples::Ping> ping_sender =
      send_event_loop.MakeSender<examples::Ping>("/test");
  {
    aos::Sender<examples::Ping>::Builder builder = ping_sender.MakeBuilder();
    examples::Ping::Builder ping_builder =
        builder.MakeBuilder<examples::Ping>();
    ping_builder.add_value(1);
    builder.CheckOk(builder.Send(ping_builder.Finish()));
  }

  MakePi1Client();

  FLAGS_application_name = "pi1_timestamp";
  aos::ShmEventLoop pi1_remote_timestamp_event_loop(&config.message());

  const size_t ping_channel_index = configuration::ChannelIndex(
      receive_event_loop.configuration(), ping_fetcher.channel());

  // ping_timestamp_count is accessed from multiple threads (the Watcher that
  // triggers it is in a separate thread), so make it atomic.
  std::atomic<int> ping_timestamp_count{0};
  const std::string channel_name =
      shared() ? "/pi1/aos/remote_timestamps/pi2"
               : "/pi1/aos/remote_timestamps/pi2/test/aos-examples-Ping";
  pi1_remote_timestamp_event_loop.MakeWatcher(
      channel_name, [this, channel_name, ping_channel_index,
                     &ping_timestamp_count](const RemoteMessage &header) {
        VLOG(1) << channel_name << " RemoteMessage "
                << aos::FlatbufferToJson(&header);
        EXPECT_TRUE(header.has_boot_uuid());
        if (shared() && header.channel_index() != ping_channel_index) {
          return;
        }
        CHECK_EQ(header.channel_index(), ping_channel_index);
        ++ping_timestamp_count;
      });

  // Before everything starts up, confirm there is no message.
  EXPECT_FALSE(ping_fetcher.Fetch());
  EXPECT_FALSE(unreliable_ping_fetcher.Fetch());

  // Spin up the persistent pieces.
  StartPi1Client();
  StartPi2Server();
  StartPi2Client();

  std::unique_ptr<ThreadedEventLoopRunner> pi1_remote_timestamp_thread =
      std::make_unique<ThreadedEventLoopRunner>(
          &pi1_remote_timestamp_event_loop);

  {
    // Now, spin up a server for 2 seconds.
    MakePi1Server();

    RunPi1Server(chrono::milliseconds(2050));

    // Confirm there is no detected duplicate packet.
    EXPECT_TRUE(pi2_client_statistics_fetcher.Fetch());
    EXPECT_EQ(pi2_client_statistics_fetcher->connections()
                  ->Get(0)
                  ->duplicate_packets(),
              0u);

    EXPECT_EQ(pi2_client_statistics_fetcher->connections()
                  ->Get(0)
                  ->partial_deliveries(),
              0u);

    EXPECT_TRUE(ping_fetcher.Fetch());
    EXPECT_FALSE(unreliable_ping_fetcher.Fetch());
    EXPECT_EQ(ping_timestamp_count, 1);
    LOG(INFO) << "Shutting down first pi1 MessageBridgeServer";

    StopPi1Server();
  }

  {
    // Now, spin up a second server for 2 seconds.
    MakePi1Server();

    RunPi1Server(chrono::milliseconds(2050));

    // Confirm we detect the duplicate packet correctly.
    EXPECT_TRUE(pi2_client_statistics_fetcher.Fetch());
    EXPECT_EQ(pi2_client_statistics_fetcher->connections()
                  ->Get(0)
                  ->duplicate_packets(),
              1u);

    EXPECT_EQ(pi2_client_statistics_fetcher->connections()
                  ->Get(0)
                  ->partial_deliveries(),
              0u);

    EXPECT_EQ(ping_timestamp_count, 1);
    EXPECT_FALSE(ping_fetcher.Fetch());
    EXPECT_FALSE(unreliable_ping_fetcher.Fetch());

    StopPi1Server();
  }

  // Shut everyone else down.
  StopPi1Client();
  StopPi2Server();
  StopPi2Client();
  pi1_remote_timestamp_thread.reset();
}

// Tests that when multiple reliable messages are sent during a time when the
// client is restarting that only the final of those messages makes it to the
// client. This ensures that we handle a disconnecting & reconnecting client
// correctly in the server reliable connection retry logic.
TEST_P(MessageBridgeParameterizedTest, ReliableSentDuringClientReboot) {
  OnPi1();

  FLAGS_application_name = "sender";
  aos::ShmEventLoop send_event_loop(&config.message());
  aos::Sender<examples::Ping> ping_sender =
      send_event_loop.MakeSender<examples::Ping>("/test");
  size_t ping_index = 0;
  SendPing(&ping_sender, ++ping_index);

  MakePi1Server();
  MakePi1Client();

  FLAGS_application_name = "pi1_timestamp";
  aos::ShmEventLoop pi1_remote_timestamp_event_loop(&config.message());

  // Now do it for "raspberrypi2", the client.
  OnPi2();

  MakePi2Server();

  aos::ShmEventLoop receive_event_loop(&config.message());
  aos::Fetcher<examples::Ping> ping_fetcher =
      receive_event_loop.MakeFetcher<examples::Ping>("/test");
  aos::Fetcher<ClientStatistics> pi2_client_statistics_fetcher =
      receive_event_loop.MakeFetcher<ClientStatistics>("/pi2/aos");

  const size_t ping_channel_index = configuration::ChannelIndex(
      receive_event_loop.configuration(), ping_fetcher.channel());

  // ping_timestamp_count is accessed from multiple threads (the Watcher that
  // triggers it is in a separate thread), so make it atomic.
  std::atomic<int> ping_timestamp_count{0};
  const std::string channel_name =
      shared() ? "/pi1/aos/remote_timestamps/pi2"
               : "/pi1/aos/remote_timestamps/pi2/test/aos-examples-Ping";
  pi1_remote_timestamp_event_loop.MakeWatcher(
      channel_name, [this, channel_name, ping_channel_index,
                     &ping_timestamp_count](const RemoteMessage &header) {
        VLOG(1) << channel_name << " RemoteMessage "
                << aos::FlatbufferToJson(&header);
        EXPECT_TRUE(header.has_boot_uuid());
        if (shared() && header.channel_index() != ping_channel_index) {
          return;
        }
        CHECK_EQ(header.channel_index(), ping_channel_index);
        ++ping_timestamp_count;
      });

  // Before everything starts up, confirm there is no message.
  EXPECT_FALSE(ping_fetcher.Fetch());

  // Spin up the persistent pieces.
  StartPi1Server();
  StartPi1Client();
  StartPi2Server();

  // Event used to wait for the timestamp counting thread to start.
  std::unique_ptr<ThreadedEventLoopRunner> pi1_remote_timestamp_thread =
      std::make_unique<ThreadedEventLoopRunner>(
          &pi1_remote_timestamp_event_loop);

  {
    // Now, spin up a client for 2 seconds.
    MakePi2Client();

    RunPi2Client(chrono::milliseconds(2050));

    // Confirm there is no detected duplicate packet.
    EXPECT_TRUE(pi2_client_statistics_fetcher.Fetch());
    EXPECT_EQ(pi2_client_statistics_fetcher->connections()
                  ->Get(0)
                  ->duplicate_packets(),
              0u);

    EXPECT_EQ(pi2_client_statistics_fetcher->connections()
                  ->Get(0)
                  ->partial_deliveries(),
              0u);

    EXPECT_TRUE(ping_fetcher.Fetch());
    EXPECT_EQ(ping_timestamp_count, 1);

    StopPi2Client();
  }

  // Send some reliable messages while the client is dead. Only the final one
  // should make it through.
  while (ping_index < 10) {
    SendPing(&ping_sender, ++ping_index);
  }

  {
    // Now, spin up a client for 2 seconds.
    MakePi2Client();

    RunPi2Client(chrono::milliseconds(5050));

    // No duplicate packets should have appeared.
    EXPECT_TRUE(pi2_client_statistics_fetcher.Fetch());
    EXPECT_EQ(pi2_client_statistics_fetcher->connections()
                  ->Get(0)
                  ->duplicate_packets(),
              0u);

    EXPECT_EQ(pi2_client_statistics_fetcher->connections()
                  ->Get(0)
                  ->partial_deliveries(),
              0u);

    EXPECT_EQ(ping_timestamp_count, 2);
    // We should have gotten precisely one more ping message--the latest one
    // sent should've made it, but no previous ones.
    EXPECT_TRUE(ping_fetcher.FetchNext());
    EXPECT_EQ(ping_index, ping_fetcher->value());
    EXPECT_FALSE(ping_fetcher.FetchNext());

    StopPi2Client();
  }

  // Shut everyone else down.
  StopPi1Client();
  StopPi2Server();
  pi1_remote_timestamp_thread.reset();
  StopPi1Server();
}

// Test that differing config sha256's result in no connection.
TEST_P(MessageBridgeParameterizedTest, MismatchedSha256) {
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
  OnPi1();

  MakePi1Server(
      "dummy sha256                                                    ");
  MakePi1Client();

  // And build the app for testing.
  MakePi1Test();
  aos::Fetcher<ServerStatistics> pi1_server_statistics_fetcher =
      pi1_test_event_loop->MakeFetcher<ServerStatistics>("/pi1/aos");
  aos::Fetcher<ClientStatistics> pi1_client_statistics_fetcher =
      pi1_test_event_loop->MakeFetcher<ClientStatistics>("/pi1/aos");

  // Now do it for "raspberrypi2", the client.
  OnPi2();
  MakePi2Server();

  // And build the app for testing.
  MakePi2Test();
  aos::Fetcher<ServerStatistics> pi2_server_statistics_fetcher =
      pi2_test_event_loop->MakeFetcher<ServerStatistics>("/pi2/aos");
  aos::Fetcher<ClientStatistics> pi2_client_statistics_fetcher =
      pi2_test_event_loop->MakeFetcher<ClientStatistics>("/pi2/aos");

  // Wait until we are connected, then send.

  StartPi1Test();
  StartPi2Test();
  StartPi1Server();
  StartPi1Client();
  StartPi2Server();

  {
    MakePi2Client();

    RunPi2Client(chrono::milliseconds(3050));

    // Now confirm we are synchronized.
    EXPECT_TRUE(pi1_server_statistics_fetcher.Fetch());
    EXPECT_TRUE(pi1_client_statistics_fetcher.Fetch());
    EXPECT_TRUE(pi2_server_statistics_fetcher.Fetch());
    EXPECT_TRUE(pi2_client_statistics_fetcher.Fetch());

    const ServerConnection *const pi1_connection =
        pi1_server_statistics_fetcher->connections()->Get(0);
    const ClientConnection *const pi1_client_connection =
        pi1_client_statistics_fetcher->connections()->Get(0);
    const ServerConnection *const pi2_connection =
        pi2_server_statistics_fetcher->connections()->Get(0);
    const ClientConnection *const pi2_client_connection =
        pi2_client_statistics_fetcher->connections()->Get(0);

    // Make sure one direction is disconnected with a bunch of connection
    // attempts and failures.
    EXPECT_EQ(pi1_connection->state(), State::DISCONNECTED);
    EXPECT_EQ(pi1_connection->connection_count(), 0u);
    EXPECT_GT(pi1_connection->invalid_connection_count(), 10u);

    EXPECT_EQ(pi2_client_connection->state(), State::DISCONNECTED);
    EXPECT_GT(pi2_client_connection->connection_count(), 10u);

    // And the other direction is happy.
    EXPECT_EQ(pi2_connection->state(), State::CONNECTED);
    EXPECT_EQ(pi2_connection->connection_count(), 1u);
    EXPECT_TRUE(pi2_connection->has_connected_since_time());
    EXPECT_FALSE(pi2_connection->has_monotonic_offset());
    EXPECT_TRUE(pi2_connection->has_boot_uuid());

    EXPECT_EQ(pi1_client_connection->state(), State::CONNECTED);
    EXPECT_EQ(pi1_client_connection->connection_count(), 1u);

    VLOG(1) << aos::FlatbufferToJson(pi2_server_statistics_fetcher.get());
    VLOG(1) << aos::FlatbufferToJson(pi1_server_statistics_fetcher.get());
    VLOG(1) << aos::FlatbufferToJson(pi2_client_statistics_fetcher.get());
    VLOG(1) << aos::FlatbufferToJson(pi1_client_statistics_fetcher.get());

    StopPi2Client();
  }

  // Shut everyone else down.
  StopPi1Server();
  StopPi1Client();
  StopPi2Server();
  StopPi1Test();
  StopPi2Test();
}

// Test that a client which connects with too big a message gets disconnected
// without crashing.
TEST_P(MessageBridgeParameterizedTest, TooBigConnect) {
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
  OnPi1();

  MakePi1Server();
  MakePi1Client();

  // And build the app for testing.
  MakePi1Test();
  aos::Fetcher<ServerStatistics> pi1_server_statistics_fetcher =
      pi1_test_event_loop->MakeFetcher<ServerStatistics>("/pi1/aos");
  aos::Fetcher<ClientStatistics> pi1_client_statistics_fetcher =
      pi1_test_event_loop->MakeFetcher<ClientStatistics>("/pi1/aos");

  // Now do it for "raspberrypi2", the client.
  OnPi2();
  MakePi2Server();

  // And build the app for testing.
  MakePi2Test();
  aos::Fetcher<ServerStatistics> pi2_server_statistics_fetcher =
      pi2_test_event_loop->MakeFetcher<ServerStatistics>("/pi2/aos");
  aos::Fetcher<ClientStatistics> pi2_client_statistics_fetcher =
      pi2_test_event_loop->MakeFetcher<ClientStatistics>("/pi2/aos");

  // Wait until we are connected, then send.

  StartPi1Test();
  StartPi2Test();
  StartPi1Server();
  StartPi1Client();
  StartPi2Server();

  {
    // Now, spin up a SctpClient and send a massive hunk of data.  This should
    // trigger a disconnect, but no crash.
    OnPi2();
    FLAGS_application_name = "pi2_message_bridge_client";
    pi2_client_event_loop =
        std::make_unique<aos::ShmEventLoop>(&config.message());
    pi2_client_event_loop->SetRuntimeRealtimePriority(1);

    const aos::Node *const remote_node = CHECK_NOTNULL(
        configuration::GetNode(pi2_client_event_loop->configuration(), "pi1"));

    const aos::FlatbufferDetachedBuffer<aos::message_bridge::Connect>
        connect_message(MakeConnectMessage(
            pi2_client_event_loop->configuration(),
            pi2_client_event_loop->node(), "pi1",
            pi2_client_event_loop->boot_uuid(), config_sha256));

    SctpClient client(remote_node->hostname()->string_view(),
                      remote_node->port(),
                      connect_message.message().channels_to_transfer()->size() +
                          kControlStreams(),
                      "");

    client.SetPoolSize(2u);

    // Passes on a machine with:
    // 5.4.0-147-generic
    // net.core.wmem_default = 212992
    // net.core.wmem_max = 212992
    // net.core.rmem_default = 212992
    // net.core.rmem_max = 212992
    // If too large it appears the message is never delivered to the
    // application.
    constexpr size_t kBigMessageSize = 64000;
    client.SetMaxReadSize(kBigMessageSize);
    client.SetMaxWriteSize(kBigMessageSize);

    const std::string big_data(kBigMessageSize, 'a');

    pi2_client_event_loop->epoll()->OnReadable(client.fd(), [&]() {
      aos::unique_c_ptr<Message> message = client.Read();
      client.FreeMessage(std::move(message));
    });

    aos::TimerHandler *const send_big_message = pi2_client_event_loop->AddTimer(
        [&]() { CHECK(client.Send(kConnectStream(), big_data, 0)); });

    pi2_client_event_loop->OnRun([this, send_big_message]() {
      send_big_message->Schedule(pi2_client_event_loop->monotonic_now() +
                                 chrono::seconds(1));
    });

    RunPi2Client(chrono::milliseconds(3050));

    // Now confirm we are synchronized.
    EXPECT_TRUE(pi1_server_statistics_fetcher.Fetch());
    EXPECT_TRUE(pi1_client_statistics_fetcher.Fetch());
    EXPECT_TRUE(pi2_server_statistics_fetcher.Fetch());
    EXPECT_FALSE(pi2_client_statistics_fetcher.Fetch());

    const ServerConnection *const pi1_connection =
        pi1_server_statistics_fetcher->connections()->Get(0);
    const ClientConnection *const pi1_client_connection =
        pi1_client_statistics_fetcher->connections()->Get(0);
    const ServerConnection *const pi2_connection =
        pi2_server_statistics_fetcher->connections()->Get(0);

    // Make sure the server we just sent a bunch of junk to is grumpy and
    // disconnected the bad client.
    EXPECT_EQ(pi1_connection->state(), State::DISCONNECTED);
    EXPECT_EQ(pi1_connection->connection_count(), 0u);
    EXPECT_GE(pi1_server_statistics_fetcher->invalid_connection_count(), 1u);

    // And the other direction is happy.
    EXPECT_EQ(pi2_connection->state(), State::CONNECTED);
    EXPECT_EQ(pi2_connection->connection_count(), 1u);
    EXPECT_TRUE(pi2_connection->has_connected_since_time());
    EXPECT_FALSE(pi2_connection->has_monotonic_offset());
    EXPECT_TRUE(pi2_connection->has_boot_uuid());

    EXPECT_EQ(pi1_client_connection->state(), State::CONNECTED);
    EXPECT_EQ(pi1_client_connection->connection_count(), 1u);

    VLOG(1) << aos::FlatbufferToJson(pi2_server_statistics_fetcher.get());
    VLOG(1) << aos::FlatbufferToJson(pi1_server_statistics_fetcher.get());
    VLOG(1) << aos::FlatbufferToJson(pi1_client_statistics_fetcher.get());

    pi2_client_event_loop->epoll()->DeleteFd(client.fd());

    StopPi2Client();
  }

  // Shut everyone else down.
  StopPi1Server();
  StopPi1Client();
  StopPi2Server();
  StopPi1Test();
  StopPi2Test();
}

INSTANTIATE_TEST_SUITE_P(
    MessageBridgeTests, MessageBridgeParameterizedTest,
    ::testing::Values(
        Param{"message_bridge_test_combined_timestamps_common_config.json",
              true},
        Param{"message_bridge_test_common_config.json", false}));

}  // namespace testing
}  // namespace message_bridge
}  // namespace aos
