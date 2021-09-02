#include "aos/events/simulated_event_loop.h"

#include <string_view>

#include "aos/events/event_loop_param_test.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/events/message_counter.h"
#include "aos/events/ping_lib.h"
#include "aos/events/pong_lib.h"
#include "aos/events/test_message_generated.h"
#include "aos/network/message_bridge_client_generated.h"
#include "aos/network/message_bridge_server_generated.h"
#include "aos/network/remote_message_generated.h"
#include "aos/network/testing_time_converter.h"
#include "aos/network/timestamp_generated.h"
#include "aos/testing/path.h"
#include "gtest/gtest.h"

namespace aos {
namespace testing {
namespace {

using aos::testing::ArtifactPath;

using logger::BootTimestamp;
using message_bridge::RemoteMessage;
namespace chrono = ::std::chrono;

}  // namespace

class SimulatedEventLoopTestFactory : public EventLoopTestFactory {
 public:
  ::std::unique_ptr<EventLoop> Make(std::string_view name) override {
    MaybeMake();
    return event_loop_factory_->MakeEventLoop(name, my_node());
  }
  ::std::unique_ptr<EventLoop> MakePrimary(std::string_view name) override {
    MaybeMake();
    return event_loop_factory_->MakeEventLoop(name, my_node());
  }

  void Run() override { event_loop_factory_->Run(); }
  void Exit() override { event_loop_factory_->Exit(); }

  // TODO(austin): Implement this.  It's used currently for a phased loop test.
  // I'm not sure how much that matters.
  void SleepFor(::std::chrono::nanoseconds /*duration*/) override {}

  void set_send_delay(std::chrono::nanoseconds send_delay) {
    MaybeMake();
    event_loop_factory_->set_send_delay(send_delay);
  }

 private:
  void MaybeMake() {
    if (!event_loop_factory_) {
      if (configuration()->has_nodes()) {
        event_loop_factory_ =
            std::make_unique<SimulatedEventLoopFactory>(configuration());
      } else {
        event_loop_factory_ =
            std::make_unique<SimulatedEventLoopFactory>(configuration());
      }
    }
  }
  std::unique_ptr<SimulatedEventLoopFactory> event_loop_factory_;
};

auto CommonParameters() {
  return ::testing::Combine(
      ::testing::Values([]() { return new SimulatedEventLoopTestFactory(); }),
      ::testing::Values(ReadMethod::COPY, ReadMethod::PIN),
      ::testing::Values(DoTimingReports::kYes, DoTimingReports::kNo));
}

INSTANTIATE_TEST_SUITE_P(SimulatedEventLoopCommonTest, AbstractEventLoopTest,
                         CommonParameters());

INSTANTIATE_TEST_SUITE_P(SimulatedEventLoopCommonDeathTest,
                         AbstractEventLoopDeathTest, CommonParameters());

// Parameters to run all the tests with.
struct Param {
  // The config file to use.
  std::string config;
  // If true, the RemoteMessage channel should be shared between all the remote
  // channels.  If false, there will be 1 RemoteMessage channel per remote
  // channel.
  bool shared;
};

class RemoteMessageSimulatedEventLoopTest
    : public ::testing::TestWithParam<struct Param> {
 public:
  RemoteMessageSimulatedEventLoopTest()
      : config(aos::configuration::ReadConfig(
            ArtifactPath(absl::StrCat("aos/events/", GetParam().config)))) {
    LOG(INFO) << "Config " << GetParam().config;
  }

  bool shared() const { return GetParam().shared; }

  std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>>
  MakePi2OnPi1MessageCounters(aos::EventLoop *event_loop) {
    std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>> counters;
    if (shared()) {
      counters.emplace_back(std::make_unique<MessageCounter<RemoteMessage>>(
          event_loop, "/aos/remote_timestamps/pi2"));
    } else {
      counters.emplace_back(std::make_unique<MessageCounter<RemoteMessage>>(
          event_loop,
          "/aos/remote_timestamps/pi2/pi1/aos/aos-message_bridge-Timestamp"));
      counters.emplace_back(std::make_unique<MessageCounter<RemoteMessage>>(
          event_loop, "/aos/remote_timestamps/pi2/test/aos-examples-Ping"));
      counters.emplace_back(std::make_unique<MessageCounter<RemoteMessage>>(
          event_loop, "/aos/remote_timestamps/pi2/reliable/aos-examples-Ping"));
    }
    return counters;
  }

  std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>>
  MakePi1OnPi2MessageCounters(aos::EventLoop *event_loop) {
    std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>> counters;
    if (shared()) {
      counters.emplace_back(std::make_unique<MessageCounter<RemoteMessage>>(
          event_loop, "/aos/remote_timestamps/pi1"));
    } else {
      counters.emplace_back(std::make_unique<MessageCounter<RemoteMessage>>(
          event_loop, "/aos/remote_timestamps/pi1/test/aos-examples-Pong"));
      counters.emplace_back(std::make_unique<MessageCounter<RemoteMessage>>(
          event_loop,
          "/aos/remote_timestamps/pi1/pi2/aos/aos-message_bridge-Timestamp"));
    }
    return counters;
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> config;
};

// Test that creating an event and running the scheduler runs the event.
TEST(EventSchedulerTest, ScheduleEvent) {
  int counter = 0;
  EventSchedulerScheduler scheduler_scheduler;
  EventScheduler scheduler(0);
  scheduler_scheduler.AddEventScheduler(&scheduler);

  scheduler.Schedule(monotonic_clock::epoch() + chrono::seconds(1),
                     [&counter]() { counter += 1; });
  scheduler_scheduler.Run();
  EXPECT_EQ(counter, 1);
  auto token = scheduler.Schedule(monotonic_clock::epoch() + chrono::seconds(2),
                                  [&counter]() { counter += 1; });
  scheduler.Deschedule(token);
  scheduler_scheduler.Run();
  EXPECT_EQ(counter, 1);
}

// Test that descheduling an already scheduled event doesn't run the event.
TEST(EventSchedulerTest, DescheduleEvent) {
  int counter = 0;
  EventSchedulerScheduler scheduler_scheduler;
  EventScheduler scheduler(0);
  scheduler_scheduler.AddEventScheduler(&scheduler);

  auto token = scheduler.Schedule(monotonic_clock::epoch() + chrono::seconds(1),
                                  [&counter]() { counter += 1; });
  scheduler.Deschedule(token);
  scheduler_scheduler.Run();
  EXPECT_EQ(counter, 0);
}

void SendTestMessage(aos::Sender<TestMessage> *sender, int value) {
  aos::Sender<TestMessage>::Builder builder = sender->MakeBuilder();
  TestMessage::Builder test_message_builder =
      builder.MakeBuilder<TestMessage>();
  test_message_builder.add_value(value);
  builder.Send(test_message_builder.Finish());
}

// Test that sending a message after running gets properly notified.
TEST(SimulatedEventLoopTest, SendAfterRunFor) {
  SimulatedEventLoopTestFactory factory;

  SimulatedEventLoopFactory simulated_event_loop_factory(
      factory.configuration());

  ::std::unique_ptr<EventLoop> ping_event_loop =
      simulated_event_loop_factory.MakeEventLoop("ping");
  aos::Sender<TestMessage> test_message_sender =
      ping_event_loop->MakeSender<TestMessage>("/test");
  SendTestMessage(&test_message_sender, 1);

  std::unique_ptr<EventLoop> pong1_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pong");
  MessageCounter<TestMessage> test_message_counter1(pong1_event_loop.get(),
                                                    "/test");

  EXPECT_FALSE(ping_event_loop->is_running());

  // Watchers start when you start running, so there should be nothing counted.
  simulated_event_loop_factory.RunFor(chrono::seconds(1));
  EXPECT_EQ(test_message_counter1.count(), 0u);

  std::unique_ptr<EventLoop> pong2_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pong");
  MessageCounter<TestMessage> test_message_counter2(pong2_event_loop.get(),
                                                    "/test");

  // Pauses in the middle don't count though, so this should be counted.
  // But, the fresh watcher shouldn't pick it up yet.
  SendTestMessage(&test_message_sender, 2);

  EXPECT_EQ(test_message_counter1.count(), 0u);
  EXPECT_EQ(test_message_counter2.count(), 0u);
  simulated_event_loop_factory.RunFor(chrono::seconds(1));

  EXPECT_EQ(test_message_counter1.count(), 1u);
  EXPECT_EQ(test_message_counter2.count(), 0u);
}

// Test that creating an event loop while running dies.
TEST(SimulatedEventLoopDeathTest, MakeEventLoopWhileRunning) {
  SimulatedEventLoopTestFactory factory;

  SimulatedEventLoopFactory simulated_event_loop_factory(
      factory.configuration());

  ::std::unique_ptr<EventLoop> event_loop =
      simulated_event_loop_factory.MakeEventLoop("ping");

  auto timer = event_loop->AddTimer([&]() {
    EXPECT_DEATH(
        {
          ::std::unique_ptr<EventLoop> event_loop2 =
              simulated_event_loop_factory.MakeEventLoop("ping");
        },
        "event loop while running");
    simulated_event_loop_factory.Exit();
  });

  event_loop->OnRun([&event_loop, &timer] {
    timer->Setup(event_loop->monotonic_now() + chrono::milliseconds(50));
  });

  simulated_event_loop_factory.Run();
}

// Test that creating a watcher after running dies.
TEST(SimulatedEventLoopDeathTest, MakeWatcherAfterRunning) {
  SimulatedEventLoopTestFactory factory;

  SimulatedEventLoopFactory simulated_event_loop_factory(
      factory.configuration());

  ::std::unique_ptr<EventLoop> event_loop =
      simulated_event_loop_factory.MakeEventLoop("ping");

  simulated_event_loop_factory.RunFor(chrono::seconds(1));

  EXPECT_DEATH(
      { MessageCounter<TestMessage> counter(event_loop.get(), "/test"); },
      "Can't add a watcher after running");

  ::std::unique_ptr<EventLoop> event_loop2 =
      simulated_event_loop_factory.MakeEventLoop("ping");

  simulated_event_loop_factory.RunFor(chrono::seconds(1));

  EXPECT_DEATH(
      { MessageCounter<TestMessage> counter(event_loop2.get(), "/test"); },
      "Can't add a watcher after running");
}

// Test that running for a time period with no handlers causes time to progress
// correctly.
TEST(SimulatedEventLoopTest, RunForNoHandlers) {
  SimulatedEventLoopTestFactory factory;

  SimulatedEventLoopFactory simulated_event_loop_factory(
      factory.configuration());
  ::std::unique_ptr<EventLoop> event_loop =
      simulated_event_loop_factory.MakeEventLoop("loop");

  simulated_event_loop_factory.RunFor(chrono::seconds(1));

  EXPECT_EQ(::aos::monotonic_clock::epoch() + chrono::seconds(1),
            event_loop->monotonic_now());
}

// Test that running for a time with a periodic handler causes time to end
// correctly.
TEST(SimulatedEventLoopTest, RunForTimerHandler) {
  SimulatedEventLoopTestFactory factory;

  SimulatedEventLoopFactory simulated_event_loop_factory(
      factory.configuration());
  ::std::unique_ptr<EventLoop> event_loop =
      simulated_event_loop_factory.MakeEventLoop("loop");

  int counter = 0;
  auto timer = event_loop->AddTimer([&counter]() { ++counter; });
  event_loop->OnRun([&event_loop, &timer] {
    timer->Setup(event_loop->monotonic_now() + chrono::milliseconds(50),
                 chrono::milliseconds(100));
  });

  simulated_event_loop_factory.RunFor(chrono::seconds(1));

  EXPECT_EQ(::aos::monotonic_clock::epoch() + chrono::seconds(1),
            event_loop->monotonic_now());
  EXPECT_EQ(counter, 10);
}

// Tests that watchers have latency in simulation.
TEST(SimulatedEventLoopTest, WatcherTimingReport) {
  SimulatedEventLoopTestFactory factory;
  factory.set_send_delay(std::chrono::microseconds(50));

  FLAGS_timing_report_ms = 1000;
  auto loop1 = factory.MakePrimary("primary");
  loop1->MakeWatcher("/test", [](const TestMessage &) {});

  auto loop2 = factory.Make("sender_loop");

  auto loop3 = factory.Make("report_fetcher");

  Fetcher<timing::Report> report_fetcher =
      loop3->MakeFetcher<timing::Report>("/aos");
  EXPECT_FALSE(report_fetcher.Fetch());

  auto sender = loop2->MakeSender<TestMessage>("/test");

  // Send 10 messages in the middle of a timing report period so we get
  // something interesting back.
  auto test_timer = loop2->AddTimer([&sender]() {
    for (int i = 0; i < 10; ++i) {
      aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
      TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
      builder.add_value(200 + i);
      ASSERT_TRUE(msg.Send(builder.Finish()));
    }
  });

  // Quit after 1 timing report, mid way through the next cycle.
  {
    auto end_timer = loop1->AddTimer([&factory]() { factory.Exit(); });
    end_timer->Setup(loop1->monotonic_now() + chrono::milliseconds(2500));
    end_timer->set_name("end");
  }

  loop1->OnRun([&test_timer, &loop1]() {
    test_timer->Setup(loop1->monotonic_now() + chrono::milliseconds(1500));
  });

  factory.Run();

  // And, since we are here, check that the timing report makes sense.
  // Start by looking for our event loop's timing.
  FlatbufferDetachedBuffer<timing::Report> primary_report =
      FlatbufferDetachedBuffer<timing::Report>::Empty();
  while (report_fetcher.FetchNext()) {
    LOG(INFO) << "Report " << FlatbufferToJson(report_fetcher.get());
    if (report_fetcher->name()->string_view() == "primary") {
      primary_report = CopyFlatBuffer(report_fetcher.get());
    }
  }

  // Check the watcher report.
  VLOG(1) << FlatbufferToJson(primary_report, {.multi_line = true});

  EXPECT_EQ(primary_report.message().name()->string_view(), "primary");

  // Just the timing report timer.
  ASSERT_NE(primary_report.message().timers(), nullptr);
  EXPECT_EQ(primary_report.message().timers()->size(), 2);

  // No phased loops
  ASSERT_EQ(primary_report.message().phased_loops(), nullptr);

  // And now confirm that the watcher received all 10 messages, and has latency.
  ASSERT_NE(primary_report.message().watchers(), nullptr);
  ASSERT_EQ(primary_report.message().watchers()->size(), 1);
  EXPECT_EQ(primary_report.message().watchers()->Get(0)->count(), 10);
  EXPECT_NEAR(
      primary_report.message().watchers()->Get(0)->wakeup_latency()->average(),
      0.00005, 1e-9);
  EXPECT_NEAR(
      primary_report.message().watchers()->Get(0)->wakeup_latency()->min(),
      0.00005, 1e-9);
  EXPECT_NEAR(
      primary_report.message().watchers()->Get(0)->wakeup_latency()->max(),
      0.00005, 1e-9);
  EXPECT_EQ(primary_report.message()
                .watchers()
                ->Get(0)
                ->wakeup_latency()
                ->standard_deviation(),
            0.0);

  EXPECT_EQ(
      primary_report.message().watchers()->Get(0)->handler_time()->average(),
      0.0);
  EXPECT_EQ(primary_report.message().watchers()->Get(0)->handler_time()->min(),
            0.0);
  EXPECT_EQ(primary_report.message().watchers()->Get(0)->handler_time()->max(),
            0.0);
  EXPECT_EQ(primary_report.message()
                .watchers()
                ->Get(0)
                ->handler_time()
                ->standard_deviation(),
            0.0);
}

size_t CountAll(
    const std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>>
        &counters) {
  size_t count = 0u;
  for (const std::unique_ptr<MessageCounter<RemoteMessage>> &counter :
       counters) {
    count += counter->count();
  }
  return count;
}

// Tests that ping and pong work when on 2 different nodes, and the message
// gateway messages are sent out as expected.
TEST_P(RemoteMessageSimulatedEventLoopTest, MultinodePingPong) {
  const Node *pi1 = configuration::GetNode(&config.message(), "pi1");
  const Node *pi2 = configuration::GetNode(&config.message(), "pi2");
  const Node *pi3 = configuration::GetNode(&config.message(), "pi3");

  SimulatedEventLoopFactory simulated_event_loop_factory(&config.message());

  std::unique_ptr<EventLoop> ping_event_loop =
      simulated_event_loop_factory.MakeEventLoop("ping", pi1);
  Ping ping(ping_event_loop.get());

  std::unique_ptr<EventLoop> pong_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pong", pi2);
  Pong pong(pong_event_loop.get());

  std::unique_ptr<EventLoop> pi2_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi2_pong_counter", pi2);
  MessageCounter<examples::Pong> pi2_pong_counter(
      pi2_pong_counter_event_loop.get(), "/test");
  aos::Fetcher<message_bridge::Timestamp> pi1_on_pi2_timestamp_fetcher =
      pi2_pong_counter_event_loop->MakeFetcher<message_bridge::Timestamp>(
          "/pi1/aos");
  aos::Fetcher<examples::Ping> ping_on_pi2_fetcher =
      pi2_pong_counter_event_loop->MakeFetcher<examples::Ping>("/test");

  std::unique_ptr<EventLoop> pi3_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi3_pong_counter", pi3);

  std::unique_ptr<EventLoop> pi1_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi1_pong_counter", pi1);
  MessageCounter<examples::Pong> pi1_pong_counter(
      pi1_pong_counter_event_loop.get(), "/test");
  aos::Fetcher<examples::Ping> ping_on_pi1_fetcher =
      pi1_pong_counter_event_loop->MakeFetcher<examples::Ping>("/test");
  aos::Fetcher<message_bridge::Timestamp> pi1_on_pi1_timestamp_fetcher =
      pi1_pong_counter_event_loop->MakeFetcher<message_bridge::Timestamp>(
          "/aos");

  // Count timestamps.
  MessageCounter<message_bridge::Timestamp> pi1_on_pi1_timestamp_counter(
      pi1_pong_counter_event_loop.get(), "/pi1/aos");
  MessageCounter<message_bridge::Timestamp> pi1_on_pi2_timestamp_counter(
      pi2_pong_counter_event_loop.get(), "/pi1/aos");
  MessageCounter<message_bridge::Timestamp> pi1_on_pi3_timestamp_counter(
      pi3_pong_counter_event_loop.get(), "/pi1/aos");
  MessageCounter<message_bridge::Timestamp> pi2_on_pi1_timestamp_counter(
      pi1_pong_counter_event_loop.get(), "/pi2/aos");
  MessageCounter<message_bridge::Timestamp> pi2_on_pi2_timestamp_counter(
      pi2_pong_counter_event_loop.get(), "/pi2/aos");
  MessageCounter<message_bridge::Timestamp> pi3_on_pi1_timestamp_counter(
      pi1_pong_counter_event_loop.get(), "/pi3/aos");
  MessageCounter<message_bridge::Timestamp> pi3_on_pi3_timestamp_counter(
      pi3_pong_counter_event_loop.get(), "/pi3/aos");

  // Count remote timestamps
  std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>>
      remote_timestamps_pi2_on_pi1 =
          MakePi2OnPi1MessageCounters(pi1_pong_counter_event_loop.get());
  std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>>
      remote_timestamps_pi1_on_pi2 =
          MakePi1OnPi2MessageCounters(pi2_pong_counter_event_loop.get());

  // Wait to let timestamp estimation start up before looking for the results.
  simulated_event_loop_factory.RunFor(chrono::milliseconds(500));

  std::unique_ptr<EventLoop> pi1_statistics_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi1_statistics_counter", pi1);
  std::unique_ptr<EventLoop> pi2_statistics_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi2_statistics_counter", pi2);
  std::unique_ptr<EventLoop> pi3_statistics_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi3_statistics_counter", pi3);

  int pi1_server_statistics_count = 0;
  pi1_statistics_counter_event_loop->MakeWatcher(
      "/pi1/aos", [&pi1_server_statistics_count](
                      const message_bridge::ServerStatistics &stats) {
        VLOG(1) << "pi1 ServerStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 2u);
        for (const message_bridge::ServerConnection *connection :
             *stats.connections()) {
          EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
          EXPECT_TRUE(connection->has_boot_uuid());
          if (connection->node()->name()->string_view() == "pi2") {
            EXPECT_GT(connection->sent_packets(), 50);
          } else if (connection->node()->name()->string_view() == "pi3") {
            EXPECT_GE(connection->sent_packets(), 5);
          } else {
            LOG(FATAL) << "Unknown connection";
          }

          EXPECT_TRUE(connection->has_monotonic_offset());
          EXPECT_EQ(connection->monotonic_offset(), 0);
        }
        ++pi1_server_statistics_count;
      });

  int pi2_server_statistics_count = 0;
  pi2_statistics_counter_event_loop->MakeWatcher(
      "/pi2/aos", [&pi2_server_statistics_count](
                      const message_bridge::ServerStatistics &stats) {
        VLOG(1) << "pi2 ServerStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 1u);

        const message_bridge::ServerConnection *connection =
            stats.connections()->Get(0);
        EXPECT_TRUE(connection->has_boot_uuid());
        EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
        EXPECT_GT(connection->sent_packets(), 50);
        EXPECT_TRUE(connection->has_monotonic_offset());
        EXPECT_EQ(connection->monotonic_offset(), 0);
        ++pi2_server_statistics_count;
      });

  int pi3_server_statistics_count = 0;
  pi3_statistics_counter_event_loop->MakeWatcher(
      "/pi3/aos", [&pi3_server_statistics_count](
                      const message_bridge::ServerStatistics &stats) {
        VLOG(1) << "pi3 ServerStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 1u);

        const message_bridge::ServerConnection *connection =
            stats.connections()->Get(0);
        EXPECT_TRUE(connection->has_boot_uuid());
        EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
        EXPECT_GE(connection->sent_packets(), 5);
        EXPECT_TRUE(connection->has_monotonic_offset());
        EXPECT_EQ(connection->monotonic_offset(), 0);
        ++pi3_server_statistics_count;
      });

  int pi1_client_statistics_count = 0;
  pi1_statistics_counter_event_loop->MakeWatcher(
      "/pi1/aos", [&pi1_client_statistics_count](
                      const message_bridge::ClientStatistics &stats) {
        VLOG(1) << "pi1 ClientStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 2u);

        for (const message_bridge::ClientConnection *connection :
             *stats.connections()) {
          EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
          if (connection->node()->name()->string_view() == "pi2") {
            EXPECT_GT(connection->received_packets(), 50);
          } else if (connection->node()->name()->string_view() == "pi3") {
            EXPECT_GE(connection->received_packets(), 5);
          } else {
            LOG(FATAL) << "Unknown connection";
          }

          EXPECT_EQ(connection->partial_deliveries(), 0);
          EXPECT_TRUE(connection->has_monotonic_offset());
          EXPECT_EQ(connection->monotonic_offset(), 150000);
        }
        ++pi1_client_statistics_count;
      });

  int pi2_client_statistics_count = 0;
  pi2_statistics_counter_event_loop->MakeWatcher(
      "/pi2/aos", [&pi2_client_statistics_count](
                      const message_bridge::ClientStatistics &stats) {
        VLOG(1) << "pi2 ClientStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 1u);

        const message_bridge::ClientConnection *connection =
            stats.connections()->Get(0);
        EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
        EXPECT_GT(connection->received_packets(), 50);
        EXPECT_EQ(connection->partial_deliveries(), 0);
        EXPECT_TRUE(connection->has_monotonic_offset());
        EXPECT_EQ(connection->monotonic_offset(), 150000);
        ++pi2_client_statistics_count;
      });

  int pi3_client_statistics_count = 0;
  pi3_statistics_counter_event_loop->MakeWatcher(
      "/pi3/aos", [&pi3_client_statistics_count](
                      const message_bridge::ClientStatistics &stats) {
        VLOG(1) << "pi3 ClientStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 1u);

        const message_bridge::ClientConnection *connection =
            stats.connections()->Get(0);
        EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
        EXPECT_GE(connection->received_packets(), 5);
        EXPECT_EQ(connection->partial_deliveries(), 0);
        EXPECT_TRUE(connection->has_monotonic_offset());
        EXPECT_EQ(connection->monotonic_offset(), 150000);
        ++pi3_client_statistics_count;
      });

  // Find the channel index for both the /pi1/aos Timestamp channel and Ping
  // channel.
  const size_t pi1_timestamp_channel =
      configuration::ChannelIndex(pi1_pong_counter_event_loop->configuration(),
                                  pi1_on_pi2_timestamp_fetcher.channel());
  const size_t ping_timestamp_channel =
      configuration::ChannelIndex(pi1_pong_counter_event_loop->configuration(),
                                  ping_on_pi2_fetcher.channel());

  for (const Channel *channel :
       *pi1_pong_counter_event_loop->configuration()->channels()) {
    VLOG(1) << "Channel "
            << configuration::ChannelIndex(
                   pi1_pong_counter_event_loop->configuration(), channel)
            << " " << configuration::CleanedChannelToString(channel);
  }

  std::unique_ptr<EventLoop> pi1_remote_timestamp =
      simulated_event_loop_factory.MakeEventLoop("pi1_remote_timestamp", pi1);

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
    // For each remote timestamp we get back, confirm that it is either a ping
    // message, or a timestamp we sent out.  Also confirm that the timestamps
    // are correct.
    pi1_remote_timestamp->MakeWatcher(
        channel.second,
        [pi1_timestamp_channel, ping_timestamp_channel, &ping_on_pi2_fetcher,
         &ping_on_pi1_fetcher, &pi1_on_pi2_timestamp_fetcher,
         &pi1_on_pi1_timestamp_fetcher, &simulated_event_loop_factory, pi2,
         channel_index = channel.first](const RemoteMessage &header) {
          VLOG(1) << aos::FlatbufferToJson(&header);
          EXPECT_TRUE(header.has_boot_uuid());
          EXPECT_EQ(UUID::FromVector(header.boot_uuid()),
                    simulated_event_loop_factory.GetNodeEventLoopFactory(pi2)
                        ->boot_uuid());

          const aos::monotonic_clock::time_point header_monotonic_sent_time(
              chrono::nanoseconds(header.monotonic_sent_time()));
          const aos::realtime_clock::time_point header_realtime_sent_time(
              chrono::nanoseconds(header.realtime_sent_time()));
          const aos::monotonic_clock::time_point header_monotonic_remote_time(
              chrono::nanoseconds(header.monotonic_remote_time()));
          const aos::realtime_clock::time_point header_realtime_remote_time(
              chrono::nanoseconds(header.realtime_remote_time()));

          if (channel_index != -1) {
            ASSERT_EQ(channel_index, header.channel_index());
          }

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
          EXPECT_EQ(pi2_context->remote_queue_index,
                    header.remote_queue_index());
          EXPECT_EQ(pi2_context->monotonic_event_time,
                    header_monotonic_sent_time);
          EXPECT_EQ(pi2_context->realtime_event_time,
                    header_realtime_sent_time);
          EXPECT_EQ(pi2_context->realtime_remote_time,
                    header_realtime_remote_time);
          EXPECT_EQ(pi2_context->monotonic_remote_time,
                    header_monotonic_remote_time);

          // Confirm the forwarded message also matches the source message.
          EXPECT_EQ(pi1_context->queue_index, header.remote_queue_index());
          EXPECT_EQ(pi1_context->monotonic_event_time,
                    header_monotonic_remote_time);
          EXPECT_EQ(pi1_context->realtime_event_time,
                    header_realtime_remote_time);
        });
  }

  simulated_event_loop_factory.RunFor(chrono::seconds(10) -
                                      chrono::milliseconds(500) +
                                      chrono::milliseconds(5));

  EXPECT_EQ(pi1_pong_counter.count(), 1001);
  EXPECT_EQ(pi2_pong_counter.count(), 1001);

  EXPECT_EQ(pi1_on_pi1_timestamp_counter.count(), 100);
  EXPECT_EQ(pi1_on_pi2_timestamp_counter.count(), 100);
  EXPECT_EQ(pi1_on_pi3_timestamp_counter.count(), 100);
  EXPECT_EQ(pi2_on_pi1_timestamp_counter.count(), 100);
  EXPECT_EQ(pi2_on_pi2_timestamp_counter.count(), 100);
  EXPECT_EQ(pi3_on_pi1_timestamp_counter.count(), 100);
  EXPECT_EQ(pi3_on_pi3_timestamp_counter.count(), 100);

  EXPECT_EQ(pi1_server_statistics_count, 10);
  EXPECT_EQ(pi2_server_statistics_count, 10);
  EXPECT_EQ(pi3_server_statistics_count, 10);

  EXPECT_EQ(pi1_client_statistics_count, 95);
  EXPECT_EQ(pi2_client_statistics_count, 95);
  EXPECT_EQ(pi3_client_statistics_count, 95);

  // Also confirm that remote timestamps are being forwarded correctly.
  EXPECT_EQ(CountAll(remote_timestamps_pi2_on_pi1), 1101);
  EXPECT_EQ(CountAll(remote_timestamps_pi1_on_pi2), 1101);
}

// Tests that an offset between nodes can be recovered and shows up in
// ServerStatistics correctly.
TEST(SimulatedEventLoopTest, MultinodePingPongWithOffset) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(ArtifactPath(
          "aos/events/multinode_pingpong_test_combined_config.json"));
  const Node *pi1 = configuration::GetNode(&config.message(), "pi1");
  const size_t pi1_index = configuration::GetNodeIndex(&config.message(), pi1);
  ASSERT_EQ(pi1_index, 0u);
  const Node *pi2 = configuration::GetNode(&config.message(), "pi2");
  const size_t pi2_index = configuration::GetNodeIndex(&config.message(), pi2);
  ASSERT_EQ(pi2_index, 1u);
  const Node *pi3 = configuration::GetNode(&config.message(), "pi3");
  const size_t pi3_index = configuration::GetNodeIndex(&config.message(), pi3);
  ASSERT_EQ(pi3_index, 2u);

  message_bridge::TestingTimeConverter time(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory simulated_event_loop_factory(&config.message());
  simulated_event_loop_factory.SetTimeConverter(&time);

  constexpr chrono::milliseconds kOffset{1501};
  time.AddNextTimestamp(
      distributed_clock::epoch(),
      {BootTimestamp::epoch(), BootTimestamp::epoch() + kOffset,
       BootTimestamp::epoch()});

  std::unique_ptr<EventLoop> ping_event_loop =
      simulated_event_loop_factory.MakeEventLoop("ping", pi1);
  Ping ping(ping_event_loop.get());

  std::unique_ptr<EventLoop> pong_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pong", pi2);
  Pong pong(pong_event_loop.get());

  // Wait to let timestamp estimation start up before looking for the results.
  simulated_event_loop_factory.RunFor(chrono::milliseconds(500));

  std::unique_ptr<EventLoop> pi1_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi1_pong_counter", pi1);

  std::unique_ptr<EventLoop> pi2_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi2_pong_counter", pi2);

  std::unique_ptr<EventLoop> pi3_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi3_pong_counter", pi3);

  // Confirm the offsets are being recovered correctly.
  int pi1_server_statistics_count = 0;
  pi1_pong_counter_event_loop->MakeWatcher(
      "/pi1/aos", [&pi1_server_statistics_count,
                   kOffset](const message_bridge::ServerStatistics &stats) {
        VLOG(1) << "pi1 ServerStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 2u);
        for (const message_bridge::ServerConnection *connection :
             *stats.connections()) {
          EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
          EXPECT_TRUE(connection->has_boot_uuid());
          if (connection->node()->name()->string_view() == "pi2") {
            EXPECT_EQ(connection->monotonic_offset(),
                      chrono::nanoseconds(kOffset).count());
          } else if (connection->node()->name()->string_view() == "pi3") {
            EXPECT_EQ(connection->monotonic_offset(), 0);
          } else {
            LOG(FATAL) << "Unknown connection";
          }

          EXPECT_TRUE(connection->has_monotonic_offset());
        }
        ++pi1_server_statistics_count;
      });

  int pi2_server_statistics_count = 0;
  pi2_pong_counter_event_loop->MakeWatcher(
      "/pi2/aos", [&pi2_server_statistics_count,
                   kOffset](const message_bridge::ServerStatistics &stats) {
        VLOG(1) << "pi2 ServerStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 1u);

        const message_bridge::ServerConnection *connection =
            stats.connections()->Get(0);
        EXPECT_TRUE(connection->has_boot_uuid());
        EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
        EXPECT_TRUE(connection->has_monotonic_offset());
        EXPECT_EQ(connection->monotonic_offset(),
                  -chrono::nanoseconds(kOffset).count());
        ++pi2_server_statistics_count;
      });

  int pi3_server_statistics_count = 0;
  pi3_pong_counter_event_loop->MakeWatcher(
      "/pi3/aos", [&pi3_server_statistics_count](
                      const message_bridge::ServerStatistics &stats) {
        VLOG(1) << "pi3 ServerStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 1u);

        const message_bridge::ServerConnection *connection =
            stats.connections()->Get(0);
        EXPECT_TRUE(connection->has_boot_uuid());
        EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
        EXPECT_TRUE(connection->has_monotonic_offset());
        EXPECT_EQ(connection->monotonic_offset(), 0);
        ++pi3_server_statistics_count;
      });

  simulated_event_loop_factory.RunFor(chrono::seconds(10) -
                                      chrono::milliseconds(500) +
                                      chrono::milliseconds(5));

  EXPECT_EQ(pi1_server_statistics_count, 10);
  EXPECT_EQ(pi2_server_statistics_count, 10);
  EXPECT_EQ(pi3_server_statistics_count, 10);
}

// Test that disabling statistics actually disables them.
TEST_P(RemoteMessageSimulatedEventLoopTest, MultinodeWithoutStatistics) {
  const Node *pi1 = configuration::GetNode(&config.message(), "pi1");
  const Node *pi2 = configuration::GetNode(&config.message(), "pi2");
  const Node *pi3 = configuration::GetNode(&config.message(), "pi3");

  SimulatedEventLoopFactory simulated_event_loop_factory(&config.message());
  simulated_event_loop_factory.DisableStatistics();

  std::unique_ptr<EventLoop> ping_event_loop =
      simulated_event_loop_factory.MakeEventLoop("ping", pi1);
  Ping ping(ping_event_loop.get());

  std::unique_ptr<EventLoop> pong_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pong", pi2);
  Pong pong(pong_event_loop.get());

  std::unique_ptr<EventLoop> pi2_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi2_pong_counter", pi2);

  MessageCounter<examples::Pong> pi2_pong_counter(
      pi2_pong_counter_event_loop.get(), "/test");

  std::unique_ptr<EventLoop> pi3_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi3_pong_counter", pi3);

  std::unique_ptr<EventLoop> pi1_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi1_pong_counter", pi1);

  MessageCounter<examples::Pong> pi1_pong_counter(
      pi1_pong_counter_event_loop.get(), "/test");

  // Count timestamps.
  MessageCounter<message_bridge::Timestamp> pi1_on_pi1_timestamp_counter(
      pi1_pong_counter_event_loop.get(), "/pi1/aos");
  MessageCounter<message_bridge::Timestamp> pi1_on_pi2_timestamp_counter(
      pi2_pong_counter_event_loop.get(), "/pi1/aos");
  MessageCounter<message_bridge::Timestamp> pi1_on_pi3_timestamp_counter(
      pi3_pong_counter_event_loop.get(), "/pi1/aos");
  MessageCounter<message_bridge::Timestamp> pi2_on_pi1_timestamp_counter(
      pi1_pong_counter_event_loop.get(), "/pi2/aos");
  MessageCounter<message_bridge::Timestamp> pi2_on_pi2_timestamp_counter(
      pi2_pong_counter_event_loop.get(), "/pi2/aos");
  MessageCounter<message_bridge::Timestamp> pi3_on_pi1_timestamp_counter(
      pi1_pong_counter_event_loop.get(), "/pi3/aos");
  MessageCounter<message_bridge::Timestamp> pi3_on_pi3_timestamp_counter(
      pi3_pong_counter_event_loop.get(), "/pi3/aos");

  // Count remote timestamps
  std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>>
      remote_timestamps_pi2_on_pi1 =
          MakePi2OnPi1MessageCounters(pi1_pong_counter_event_loop.get());
  std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>>
      remote_timestamps_pi1_on_pi2 =
          MakePi1OnPi2MessageCounters(pi2_pong_counter_event_loop.get());

  MessageCounter<message_bridge::ServerStatistics>
      pi1_server_statistics_counter(pi1_pong_counter_event_loop.get(),
                                    "/pi1/aos");
  MessageCounter<message_bridge::ServerStatistics>
      pi2_server_statistics_counter(pi2_pong_counter_event_loop.get(),
                                    "/pi2/aos");
  MessageCounter<message_bridge::ServerStatistics>
      pi3_server_statistics_counter(pi3_pong_counter_event_loop.get(),
                                    "/pi3/aos");

  MessageCounter<message_bridge::ClientStatistics>
      pi1_client_statistics_counter(pi1_pong_counter_event_loop.get(),
                                    "/pi1/aos");
  MessageCounter<message_bridge::ClientStatistics>
      pi2_client_statistics_counter(pi2_pong_counter_event_loop.get(),
                                    "/pi2/aos");
  MessageCounter<message_bridge::ClientStatistics>
      pi3_client_statistics_counter(pi3_pong_counter_event_loop.get(),
                                    "/pi3/aos");

  simulated_event_loop_factory.RunFor(chrono::seconds(10) +
                                      chrono::milliseconds(5));

  EXPECT_EQ(pi1_pong_counter.count(), 1001u);
  EXPECT_EQ(pi2_pong_counter.count(), 1001u);

  EXPECT_EQ(pi1_on_pi1_timestamp_counter.count(), 0u);
  EXPECT_EQ(pi1_on_pi2_timestamp_counter.count(), 0u);
  EXPECT_EQ(pi1_on_pi3_timestamp_counter.count(), 0u);
  EXPECT_EQ(pi2_on_pi1_timestamp_counter.count(), 0u);
  EXPECT_EQ(pi2_on_pi2_timestamp_counter.count(), 0u);
  EXPECT_EQ(pi3_on_pi1_timestamp_counter.count(), 0u);
  EXPECT_EQ(pi3_on_pi3_timestamp_counter.count(), 0u);

  EXPECT_EQ(pi1_server_statistics_counter.count(), 0u);
  EXPECT_EQ(pi2_server_statistics_counter.count(), 0u);
  EXPECT_EQ(pi3_server_statistics_counter.count(), 0u);

  EXPECT_EQ(pi1_client_statistics_counter.count(), 0u);
  EXPECT_EQ(pi2_client_statistics_counter.count(), 0u);
  EXPECT_EQ(pi3_client_statistics_counter.count(), 0u);

  // Also confirm that remote timestamps are being forwarded correctly.
  EXPECT_EQ(CountAll(remote_timestamps_pi2_on_pi1), 1001);
  EXPECT_EQ(CountAll(remote_timestamps_pi1_on_pi2), 1001);
}

bool AllConnected(const message_bridge::ServerStatistics *server_statistics) {
  for (const message_bridge::ServerConnection *connection :
       *server_statistics->connections()) {
    if (connection->state() != message_bridge::State::CONNECTED) {
      return false;
    }
  }
  return true;
}

bool AllConnectedBut(const message_bridge::ServerStatistics *server_statistics,
                     std::string_view target) {
  for (const message_bridge::ServerConnection *connection :
       *server_statistics->connections()) {
    if (connection->node()->name()->string_view() == target) {
      if (connection->state() == message_bridge::State::CONNECTED) {
        return false;
      }
    } else {
      if (connection->state() != message_bridge::State::CONNECTED) {
        return false;
      }
    }
  }
  return true;
}

bool AllConnected(const message_bridge::ClientStatistics *client_statistics) {
  for (const message_bridge::ClientConnection *connection :
       *client_statistics->connections()) {
    if (connection->state() != message_bridge::State::CONNECTED) {
      return false;
    }
  }
  return true;
}

bool AllConnectedBut(const message_bridge::ClientStatistics *client_statistics,
                     std::string_view target) {
  for (const message_bridge::ClientConnection *connection :
       *client_statistics->connections()) {
    if (connection->node()->name()->string_view() == target) {
      if (connection->state() == message_bridge::State::CONNECTED) {
        return false;
      }
    } else {
      if (connection->state() != message_bridge::State::CONNECTED) {
        return false;
      }
    }
  }
  return true;
}

// Test that disconnecting nodes actually disconnects them.
TEST_P(RemoteMessageSimulatedEventLoopTest, MultinodeDisconnect) {
  SimulatedEventLoopFactory simulated_event_loop_factory(&config.message());

  NodeEventLoopFactory *pi1 =
      simulated_event_loop_factory.GetNodeEventLoopFactory("pi1");
  NodeEventLoopFactory *pi2 =
      simulated_event_loop_factory.GetNodeEventLoopFactory("pi2");
  NodeEventLoopFactory *pi3 =
      simulated_event_loop_factory.GetNodeEventLoopFactory("pi3");

  std::unique_ptr<EventLoop> ping_event_loop = pi1->MakeEventLoop("ping");
  Ping ping(ping_event_loop.get());

  std::unique_ptr<EventLoop> pong_event_loop = pi2->MakeEventLoop("pong");
  Pong pong(pong_event_loop.get());

  std::unique_ptr<EventLoop> pi2_pong_counter_event_loop =
      pi2->MakeEventLoop("pi2_pong_counter");

  MessageCounter<examples::Pong> pi2_pong_counter(
      pi2_pong_counter_event_loop.get(), "/test");

  std::unique_ptr<EventLoop> pi3_pong_counter_event_loop =
      pi3->MakeEventLoop("pi3_pong_counter");

  std::unique_ptr<EventLoop> pi1_pong_counter_event_loop =
      pi1->MakeEventLoop("pi1_pong_counter");

  MessageCounter<examples::Pong> pi1_pong_counter(
      pi1_pong_counter_event_loop.get(), "/test");

  // Count timestamps.
  MessageCounter<message_bridge::Timestamp> pi1_on_pi1_timestamp_counter(
      pi1_pong_counter_event_loop.get(), "/pi1/aos");
  MessageCounter<message_bridge::Timestamp> pi1_on_pi2_timestamp_counter(
      pi2_pong_counter_event_loop.get(), "/pi1/aos");
  MessageCounter<message_bridge::Timestamp> pi1_on_pi3_timestamp_counter(
      pi3_pong_counter_event_loop.get(), "/pi1/aos");
  MessageCounter<message_bridge::Timestamp> pi2_on_pi1_timestamp_counter(
      pi1_pong_counter_event_loop.get(), "/pi2/aos");
  MessageCounter<message_bridge::Timestamp> pi2_on_pi2_timestamp_counter(
      pi2_pong_counter_event_loop.get(), "/pi2/aos");
  MessageCounter<message_bridge::Timestamp> pi3_on_pi1_timestamp_counter(
      pi1_pong_counter_event_loop.get(), "/pi3/aos");
  MessageCounter<message_bridge::Timestamp> pi3_on_pi3_timestamp_counter(
      pi3_pong_counter_event_loop.get(), "/pi3/aos");

  // Count remote timestamps
  std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>>
      remote_timestamps_pi2_on_pi1 =
          MakePi2OnPi1MessageCounters(pi1_pong_counter_event_loop.get());
  std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>>
      remote_timestamps_pi1_on_pi2 =
          MakePi1OnPi2MessageCounters(pi2_pong_counter_event_loop.get());

  MessageCounter<message_bridge::ServerStatistics>
      *pi1_server_statistics_counter;
  pi1->OnStartup([pi1, &pi1_server_statistics_counter]() {
    pi1_server_statistics_counter =
        pi1->AlwaysStart<MessageCounter<message_bridge::ServerStatistics>>(
            "pi1_server_statistics_counter", "/pi1/aos");
  });

  aos::Fetcher<message_bridge::ServerStatistics> pi1_server_statistics_fetcher =
      pi1_pong_counter_event_loop
          ->MakeFetcher<message_bridge::ServerStatistics>("/pi1/aos");
  aos::Fetcher<message_bridge::ClientStatistics> pi1_client_statistics_fetcher =
      pi1_pong_counter_event_loop
          ->MakeFetcher<message_bridge::ClientStatistics>("/pi1/aos");

  MessageCounter<message_bridge::ServerStatistics>
      *pi2_server_statistics_counter;
  pi2->OnStartup([pi2, &pi2_server_statistics_counter]() {
    pi2_server_statistics_counter =
        pi2->AlwaysStart<MessageCounter<message_bridge::ServerStatistics>>(
            "pi2_server_statistics_counter", "/pi2/aos");
  });
  aos::Fetcher<message_bridge::ServerStatistics> pi2_server_statistics_fetcher =
      pi2_pong_counter_event_loop
          ->MakeFetcher<message_bridge::ServerStatistics>("/pi2/aos");
  aos::Fetcher<message_bridge::ClientStatistics> pi2_client_statistics_fetcher =
      pi2_pong_counter_event_loop
          ->MakeFetcher<message_bridge::ClientStatistics>("/pi2/aos");

  MessageCounter<message_bridge::ServerStatistics>
      *pi3_server_statistics_counter;
  pi3->OnStartup([pi3, &pi3_server_statistics_counter]() {
    pi3_server_statistics_counter =
        pi3->AlwaysStart<MessageCounter<message_bridge::ServerStatistics>>(
            "pi3_server_statistics_counter", "/pi3/aos");
  });
  aos::Fetcher<message_bridge::ServerStatistics> pi3_server_statistics_fetcher =
      pi3_pong_counter_event_loop
          ->MakeFetcher<message_bridge::ServerStatistics>("/pi3/aos");
  aos::Fetcher<message_bridge::ClientStatistics> pi3_client_statistics_fetcher =
      pi3_pong_counter_event_loop
          ->MakeFetcher<message_bridge::ClientStatistics>("/pi3/aos");

  MessageCounter<message_bridge::ClientStatistics>
      pi1_client_statistics_counter(pi1_pong_counter_event_loop.get(),
                                    "/pi1/aos");
  MessageCounter<message_bridge::ClientStatistics>
      pi2_client_statistics_counter(pi2_pong_counter_event_loop.get(),
                                    "/pi2/aos");
  MessageCounter<message_bridge::ClientStatistics>
      pi3_client_statistics_counter(pi3_pong_counter_event_loop.get(),
                                    "/pi3/aos");

  simulated_event_loop_factory.RunFor(chrono::seconds(2) +
                                      chrono::milliseconds(5));

  EXPECT_EQ(pi1_pong_counter.count(), 201u);
  EXPECT_EQ(pi2_pong_counter.count(), 201u);

  EXPECT_EQ(pi1_on_pi1_timestamp_counter.count(), 20u);
  EXPECT_EQ(pi1_on_pi2_timestamp_counter.count(), 20u);
  EXPECT_EQ(pi1_on_pi3_timestamp_counter.count(), 20u);
  EXPECT_EQ(pi2_on_pi1_timestamp_counter.count(), 20u);
  EXPECT_EQ(pi2_on_pi2_timestamp_counter.count(), 20u);
  EXPECT_EQ(pi3_on_pi1_timestamp_counter.count(), 20u);
  EXPECT_EQ(pi3_on_pi3_timestamp_counter.count(), 20u);

  EXPECT_EQ(pi1_server_statistics_counter->count(), 2u);
  EXPECT_EQ(pi2_server_statistics_counter->count(), 2u);
  EXPECT_EQ(pi3_server_statistics_counter->count(), 2u);

  EXPECT_EQ(pi1_client_statistics_counter.count(), 20u);
  EXPECT_EQ(pi2_client_statistics_counter.count(), 20u);
  EXPECT_EQ(pi3_client_statistics_counter.count(), 20u);

  // Also confirm that remote timestamps are being forwarded correctly.
  EXPECT_EQ(CountAll(remote_timestamps_pi2_on_pi1), 221);
  EXPECT_EQ(CountAll(remote_timestamps_pi1_on_pi2), 221);

  EXPECT_TRUE(pi1_server_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi1_server_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi1_server_statistics_fetcher.get());
  EXPECT_TRUE(pi1_client_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi1_client_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi1_client_statistics_fetcher.get());
  EXPECT_TRUE(pi2_server_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi2_server_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi2_server_statistics_fetcher.get());
  EXPECT_TRUE(pi2_client_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi2_client_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi2_client_statistics_fetcher.get());
  EXPECT_TRUE(pi3_server_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi3_server_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi3_server_statistics_fetcher.get());
  EXPECT_TRUE(pi3_client_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi3_client_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi3_client_statistics_fetcher.get());

  pi1->Disconnect(pi3->node());

  simulated_event_loop_factory.RunFor(chrono::seconds(2));

  EXPECT_EQ(pi1_pong_counter.count(), 401u);
  EXPECT_EQ(pi2_pong_counter.count(), 401u);

  EXPECT_EQ(pi1_on_pi1_timestamp_counter.count(), 40u);
  EXPECT_EQ(pi1_on_pi2_timestamp_counter.count(), 40u);
  EXPECT_EQ(pi1_on_pi3_timestamp_counter.count(), 20u);
  EXPECT_EQ(pi2_on_pi1_timestamp_counter.count(), 40u);
  EXPECT_EQ(pi2_on_pi2_timestamp_counter.count(), 40u);
  EXPECT_EQ(pi3_on_pi1_timestamp_counter.count(), 40u);
  EXPECT_EQ(pi3_on_pi3_timestamp_counter.count(), 40u);

  EXPECT_EQ(pi1_server_statistics_counter->count(), 4u);
  EXPECT_EQ(pi2_server_statistics_counter->count(), 4u);
  EXPECT_EQ(pi3_server_statistics_counter->count(), 4u);

  EXPECT_EQ(pi1_client_statistics_counter.count(), 40u);
  EXPECT_EQ(pi2_client_statistics_counter.count(), 40u);
  EXPECT_EQ(pi3_client_statistics_counter.count(), 40u);

  // Also confirm that remote timestamps are being forwarded correctly.
  EXPECT_EQ(CountAll(remote_timestamps_pi2_on_pi1), 441);
  EXPECT_EQ(CountAll(remote_timestamps_pi1_on_pi2), 441);

  EXPECT_TRUE(pi1_server_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnectedBut(pi1_server_statistics_fetcher.get(), "pi3"))
      << " : " << aos::FlatbufferToJson(pi1_server_statistics_fetcher.get());
  EXPECT_TRUE(pi1_client_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi1_client_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi1_client_statistics_fetcher.get());
  EXPECT_TRUE(pi2_server_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi2_server_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi2_server_statistics_fetcher.get());
  EXPECT_TRUE(pi2_client_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi2_client_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi2_client_statistics_fetcher.get());
  EXPECT_TRUE(pi3_server_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi3_server_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi3_server_statistics_fetcher.get());
  EXPECT_TRUE(pi3_client_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnectedBut(pi3_client_statistics_fetcher.get(), "pi1"))
      << " : " << aos::FlatbufferToJson(pi3_client_statistics_fetcher.get());

  pi1->Connect(pi3->node());

  simulated_event_loop_factory.RunFor(chrono::seconds(2));

  EXPECT_EQ(pi1_pong_counter.count(), 601u);
  EXPECT_EQ(pi2_pong_counter.count(), 601u);

  EXPECT_EQ(pi1_on_pi1_timestamp_counter.count(), 60u);
  EXPECT_EQ(pi1_on_pi2_timestamp_counter.count(), 60u);
  EXPECT_EQ(pi1_on_pi3_timestamp_counter.count(), 40u);
  EXPECT_EQ(pi2_on_pi1_timestamp_counter.count(), 60u);
  EXPECT_EQ(pi2_on_pi2_timestamp_counter.count(), 60u);
  EXPECT_EQ(pi3_on_pi1_timestamp_counter.count(), 60u);
  EXPECT_EQ(pi3_on_pi3_timestamp_counter.count(), 60u);

  EXPECT_EQ(pi1_server_statistics_counter->count(), 6u);
  EXPECT_EQ(pi2_server_statistics_counter->count(), 6u);
  EXPECT_EQ(pi3_server_statistics_counter->count(), 6u);

  EXPECT_EQ(pi1_client_statistics_counter.count(), 60u);
  EXPECT_EQ(pi2_client_statistics_counter.count(), 60u);
  EXPECT_EQ(pi3_client_statistics_counter.count(), 60u);

  // Also confirm that remote timestamps are being forwarded correctly.
  EXPECT_EQ(CountAll(remote_timestamps_pi2_on_pi1), 661);
  EXPECT_EQ(CountAll(remote_timestamps_pi1_on_pi2), 661);

  EXPECT_TRUE(pi1_server_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi1_server_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi1_server_statistics_fetcher.get());
  EXPECT_TRUE(pi1_client_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi1_client_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi1_client_statistics_fetcher.get());
  EXPECT_TRUE(pi2_server_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi2_server_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi2_server_statistics_fetcher.get());
  EXPECT_TRUE(pi2_client_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi2_client_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi2_client_statistics_fetcher.get());
  EXPECT_TRUE(pi3_server_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi3_server_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi3_server_statistics_fetcher.get());
  EXPECT_TRUE(pi3_client_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi3_client_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi3_client_statistics_fetcher.get());
}

// Tests that the time offset having a slope doesn't break the world.
// SimulatedMessageBridge has enough self consistency CHECK statements to
// confirm, and we can can also check a message in each direction to make sure
// it gets delivered as expected.
TEST(SimulatedEventLoopTest, MultinodePingPongWithOffsetAndSlope) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(ArtifactPath(
          "aos/events/multinode_pingpong_test_combined_config.json"));
  const Node *pi1 = configuration::GetNode(&config.message(), "pi1");
  const size_t pi1_index = configuration::GetNodeIndex(&config.message(), pi1);
  ASSERT_EQ(pi1_index, 0u);
  const Node *pi2 = configuration::GetNode(&config.message(), "pi2");
  const size_t pi2_index = configuration::GetNodeIndex(&config.message(), pi2);
  ASSERT_EQ(pi2_index, 1u);
  const Node *pi3 = configuration::GetNode(&config.message(), "pi3");
  const size_t pi3_index = configuration::GetNodeIndex(&config.message(), pi3);
  ASSERT_EQ(pi3_index, 2u);

  message_bridge::TestingTimeConverter time(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory simulated_event_loop_factory(&config.message());
  simulated_event_loop_factory.SetTimeConverter(&time);

  constexpr chrono::milliseconds kOffset{150100};
  time.AddNextTimestamp(
      distributed_clock::epoch(),
      {BootTimestamp::epoch(), BootTimestamp::epoch() + kOffset,
       BootTimestamp::epoch()});
  time.AddNextTimestamp(distributed_clock::epoch() + chrono::seconds(10),
                        {BootTimestamp::epoch() + chrono::milliseconds(9999),
                         BootTimestamp::epoch() + kOffset + chrono::seconds(10),
                         BootTimestamp::epoch() + chrono::milliseconds(9999)});

  std::unique_ptr<EventLoop> ping_event_loop =
      simulated_event_loop_factory.MakeEventLoop("ping", pi1);
  Ping ping(ping_event_loop.get());

  std::unique_ptr<EventLoop> pong_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pong", pi2);
  Pong pong(pong_event_loop.get());

  std::unique_ptr<EventLoop> pi1_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi1_counter", pi1);
  std::unique_ptr<EventLoop> pi2_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi2_counter", pi2);

  aos::Fetcher<examples::Ping> ping_on_pi1_fetcher =
      pi1_counter_event_loop->MakeFetcher<examples::Ping>("/test");
  aos::Fetcher<examples::Ping> ping_on_pi2_fetcher =
      pi2_counter_event_loop->MakeFetcher<examples::Ping>("/test");

  aos::Fetcher<examples::Pong> pong_on_pi2_fetcher =
      pi2_counter_event_loop->MakeFetcher<examples::Pong>("/test");
  aos::Fetcher<examples::Pong> pong_on_pi1_fetcher =
      pi1_counter_event_loop->MakeFetcher<examples::Pong>("/test");

  // End after a pong message comes back.  This will leave the latest messages
  // on all channels so we can look at timestamps easily and check they make
  // sense.
  std::unique_ptr<EventLoop> pi1_pong_ender =
      simulated_event_loop_factory.MakeEventLoop("pi2_counter", pi1);
  int count = 0;
  pi1_pong_ender->MakeWatcher(
      "/test", [&simulated_event_loop_factory, &count](const examples::Pong &) {
        if (++count == 100) {
          simulated_event_loop_factory.Exit();
        }
      });

  // Run enough that messages should be delivered.
  simulated_event_loop_factory.Run();

  // Grab the latest messages.
  EXPECT_TRUE(ping_on_pi1_fetcher.Fetch());
  EXPECT_TRUE(ping_on_pi2_fetcher.Fetch());
  EXPECT_TRUE(pong_on_pi1_fetcher.Fetch());
  EXPECT_TRUE(pong_on_pi2_fetcher.Fetch());

  // Compute their time on the global distributed clock so we can compute
  // distance betwen them.
  const distributed_clock::time_point pi1_ping_time =
      simulated_event_loop_factory.GetNodeEventLoopFactory(pi1)
          ->ToDistributedClock(
              ping_on_pi1_fetcher.context().monotonic_event_time);
  const distributed_clock::time_point pi2_ping_time =
      simulated_event_loop_factory.GetNodeEventLoopFactory(pi2)
          ->ToDistributedClock(
              ping_on_pi2_fetcher.context().monotonic_event_time);
  const distributed_clock::time_point pi1_pong_time =
      simulated_event_loop_factory.GetNodeEventLoopFactory(pi1)
          ->ToDistributedClock(
              pong_on_pi1_fetcher.context().monotonic_event_time);
  const distributed_clock::time_point pi2_pong_time =
      simulated_event_loop_factory.GetNodeEventLoopFactory(pi2)
          ->ToDistributedClock(
              pong_on_pi2_fetcher.context().monotonic_event_time);

  // And confirm the delivery delay is just about exactly 150 uS for both
  // directions like expected.  There will be a couple ns of rounding errors in
  // the conversion functions that aren't worth accounting for right now.  This
  // will either be really close, or really far.
  EXPECT_GE(pi2_ping_time, chrono::microseconds(150) - chrono::nanoseconds(10) +
                               pi1_ping_time);
  EXPECT_LE(pi2_ping_time, chrono::microseconds(150) + chrono::nanoseconds(10) +
                               pi1_ping_time);

  EXPECT_GE(pi1_pong_time, chrono::microseconds(150) - chrono::nanoseconds(10) +
                               pi2_pong_time);
  EXPECT_LE(pi1_pong_time, chrono::microseconds(150) + chrono::nanoseconds(10) +
                               pi2_pong_time);
}

void SendPing(aos::Sender<examples::Ping> *sender, int value) {
  aos::Sender<examples::Ping>::Builder builder = sender->MakeBuilder();
  examples::Ping::Builder ping_builder = builder.MakeBuilder<examples::Ping>();
  ping_builder.add_value(value);
  builder.Send(ping_builder.Finish());
}

// Tests that reliable (and unreliable) ping messages get forwarded as expected.
TEST_P(RemoteMessageSimulatedEventLoopTest, MultinodeStartupTesting) {
  const Node *pi1 = configuration::GetNode(&config.message(), "pi1");
  const Node *pi2 = configuration::GetNode(&config.message(), "pi2");

  SimulatedEventLoopFactory simulated_event_loop_factory(&config.message());

  std::unique_ptr<EventLoop> ping_event_loop =
      simulated_event_loop_factory.MakeEventLoop("ping", pi1);
  aos::Sender<examples::Ping> pi1_reliable_sender =
      ping_event_loop->MakeSender<examples::Ping>("/reliable");
  aos::Sender<examples::Ping> pi1_unreliable_sender =
      ping_event_loop->MakeSender<examples::Ping>("/unreliable");
  SendPing(&pi1_reliable_sender, 1);
  SendPing(&pi1_unreliable_sender, 1);

  std::unique_ptr<EventLoop> pi2_pong_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pong", pi2);
  MessageCounter<examples::Ping> pi2_reliable_counter(pi2_pong_event_loop.get(),
                                                      "/reliable");
  MessageCounter<examples::Ping> pi2_unreliable_counter(
      pi2_pong_event_loop.get(), "/unreliable");
  aos::Fetcher<examples::Ping> reliable_on_pi2_fetcher =
      pi2_pong_event_loop->MakeFetcher<examples::Ping>("/reliable");
  aos::Fetcher<examples::Ping> unreliable_on_pi2_fetcher =
      pi2_pong_event_loop->MakeFetcher<examples::Ping>("/unreliable");

  const size_t reliable_channel_index = configuration::ChannelIndex(
      pi2_pong_event_loop->configuration(), reliable_on_pi2_fetcher.channel());

  std::unique_ptr<EventLoop> pi1_remote_timestamp =
      simulated_event_loop_factory.MakeEventLoop("pi1_remote_timestamp", pi1);

  const chrono::nanoseconds network_delay =
      simulated_event_loop_factory.network_delay();

  int reliable_timestamp_count = 0;
  pi1_remote_timestamp->MakeWatcher(
      shared() ? "/pi1/aos/remote_timestamps/pi2"
               : "/pi1/aos/remote_timestamps/pi2/reliable/aos-examples-Ping",
      [reliable_channel_index, &reliable_timestamp_count,
       &simulated_event_loop_factory, pi2, network_delay, &pi2_pong_event_loop,
       &pi1_remote_timestamp](const RemoteMessage &header) {
        EXPECT_TRUE(header.has_boot_uuid());
        EXPECT_EQ(UUID::FromVector(header.boot_uuid()),
                  simulated_event_loop_factory.GetNodeEventLoopFactory(pi2)
                      ->boot_uuid());
        VLOG(1) << aos::FlatbufferToJson(&header);
        if (header.channel_index() == reliable_channel_index) {
          ++reliable_timestamp_count;
        }

        const aos::monotonic_clock::time_point header_monotonic_sent_time(
            chrono::nanoseconds(header.monotonic_sent_time()));

        EXPECT_EQ(pi1_remote_timestamp->context().monotonic_event_time,
                  header_monotonic_sent_time + network_delay +
                      (pi1_remote_timestamp->monotonic_now() -
                       pi2_pong_event_loop->monotonic_now()));
      });

  // Wait to let timestamp estimation start up before looking for the results.
  simulated_event_loop_factory.RunFor(chrono::milliseconds(500));

  EXPECT_EQ(pi2_reliable_counter.count(), 1u);
  // This one isn't reliable, but was sent before the start.  It should *not* be
  // delivered.
  EXPECT_EQ(pi2_unreliable_counter.count(), 0u);
  // Confirm we got a timestamp logged for the message that was forwarded.
  EXPECT_EQ(reliable_timestamp_count, 1u);

  SendPing(&pi1_reliable_sender, 2);
  SendPing(&pi1_unreliable_sender, 2);
  simulated_event_loop_factory.RunFor(chrono::milliseconds(500));
  EXPECT_EQ(pi2_reliable_counter.count(), 2u);
  EXPECT_EQ(pi2_unreliable_counter.count(), 1u);

  EXPECT_EQ(reliable_timestamp_count, 2u);
}

// Tests that rebooting a node changes the ServerStatistics message and the
// RemoteTimestamp message.
TEST_P(RemoteMessageSimulatedEventLoopTest, BootUUIDTest) {
  const UUID pi1_boot0 = UUID::Random();
  const UUID pi2_boot0 = UUID::Random();
  const UUID pi2_boot1 = UUID::Random();
  const UUID pi3_boot0 = UUID::Random();
  UUID expected_boot_uuid = pi2_boot0;

  message_bridge::TestingTimeConverter time(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory factory(&config.message());
  factory.SetTimeConverter(&time);

  const size_t pi1_index =
      configuration::GetNodeIndex(&config.message(), "pi1");
  const size_t pi2_index =
      configuration::GetNodeIndex(&config.message(), "pi2");
  const size_t pi3_index =
      configuration::GetNodeIndex(&config.message(), "pi3");

  {
    time.AddNextTimestamp(distributed_clock::epoch(),
                          {BootTimestamp::epoch(), BootTimestamp::epoch(),
                           BootTimestamp::epoch()});

    const chrono::nanoseconds dt = chrono::milliseconds(2001);

    time.AddNextTimestamp(
        distributed_clock::epoch() + dt,
        {BootTimestamp::epoch() + dt,
         BootTimestamp{.boot = 1, .time = monotonic_clock::epoch()},
         BootTimestamp::epoch() + dt});

    time.set_boot_uuid(pi1_index, 0, pi1_boot0);
    time.set_boot_uuid(pi2_index, 0, pi2_boot0);
    time.set_boot_uuid(pi2_index, 1, pi2_boot1);
    time.set_boot_uuid(pi3_index, 0, pi3_boot0);
  }

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");
  NodeEventLoopFactory *pi2 = factory.GetNodeEventLoopFactory("pi2");

  pi1->OnStartup([pi1]() { pi1->AlwaysStart<Ping>("ping"); });
  pi2->OnStartup([pi2]() { pi2->AlwaysStart<Pong>("pong"); });

  std::unique_ptr<EventLoop> pi1_remote_timestamp =
      pi1->MakeEventLoop("pi1_remote_timestamp");

  int timestamp_count = 0;
  pi1_remote_timestamp->MakeWatcher(
      "/pi2/aos", [&expected_boot_uuid,
                   &pi1_remote_timestamp](const message_bridge::Timestamp &) {
        EXPECT_EQ(pi1_remote_timestamp->context().source_boot_uuid,
                  expected_boot_uuid);
      });
  pi1_remote_timestamp->MakeWatcher(
      "/test",
      [&expected_boot_uuid, &pi1_remote_timestamp](const examples::Pong &) {
        EXPECT_EQ(pi1_remote_timestamp->context().source_boot_uuid,
                  expected_boot_uuid);
      });
  pi1_remote_timestamp->MakeWatcher(
      shared() ? "/pi1/aos/remote_timestamps/pi2"
               : "/pi1/aos/remote_timestamps/pi2/test/aos-examples-Ping",
      [&timestamp_count, &expected_boot_uuid](const RemoteMessage &header) {
        EXPECT_TRUE(header.has_boot_uuid());
        EXPECT_EQ(UUID::FromVector(header.boot_uuid()), expected_boot_uuid);
        VLOG(1) << aos::FlatbufferToJson(&header);
        ++timestamp_count;
      });

  int pi1_server_statistics_count = 0;
  bool first_pi1_server_statistics = true;
  pi1_remote_timestamp->MakeWatcher(
      "/pi1/aos", [&pi1_server_statistics_count, &expected_boot_uuid,
                   &first_pi1_server_statistics](
                      const message_bridge::ServerStatistics &stats) {
        VLOG(1) << "pi1 ServerStatistics " << FlatbufferToJson(&stats);
        for (const message_bridge::ServerConnection *connection :
             *stats.connections()) {
          if (connection->state() == message_bridge::State::CONNECTED) {
            ASSERT_TRUE(connection->has_boot_uuid());
          }
          if (!first_pi1_server_statistics) {
            EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
          }
          if (connection->node()->name()->string_view() == "pi2") {
            EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
            ASSERT_TRUE(connection->has_boot_uuid());
            EXPECT_EQ(expected_boot_uuid,
                      UUID::FromString(connection->boot_uuid()))
                << " : Got " << aos::FlatbufferToJson(&stats);
            ++pi1_server_statistics_count;
          }
        }
        first_pi1_server_statistics = false;
      });

  int pi1_client_statistics_count = 0;
  pi1_remote_timestamp->MakeWatcher(
      "/pi1/aos", [&pi1_client_statistics_count](
                      const message_bridge::ClientStatistics &stats) {
        VLOG(1) << "pi1 ClientStatistics " << FlatbufferToJson(&stats);
        for (const message_bridge::ClientConnection *connection :
             *stats.connections()) {
          EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
          if (connection->node()->name()->string_view() == "pi2") {
            ++pi1_client_statistics_count;
          }
        }
      });

  // Confirm that reboot changes the UUID.
  pi2->OnShutdown([&expected_boot_uuid, pi2, pi2_boot1]() {
    expected_boot_uuid = pi2_boot1;
    LOG(INFO) << "OnShutdown triggered for pi2";
    pi2->OnStartup([&expected_boot_uuid, pi2]() {
      EXPECT_EQ(expected_boot_uuid, pi2->boot_uuid());
    });
  });

  // Let a couple of ServerStatistics messages show up before rebooting.
  factory.RunFor(chrono::milliseconds(2002));

  EXPECT_GT(timestamp_count, 100);
  EXPECT_GE(pi1_server_statistics_count, 1u);

  timestamp_count = 0;
  pi1_server_statistics_count = 0;

  factory.RunFor(chrono::milliseconds(2000));
  EXPECT_GT(timestamp_count, 100);
  EXPECT_GE(pi1_server_statistics_count, 1u);
}

INSTANTIATE_TEST_SUITE_P(
    All, RemoteMessageSimulatedEventLoopTest,
    ::testing::Values(
        Param{"multinode_pingpong_test_combined_config.json", true},
        Param{"multinode_pingpong_test_split_config.json", false}));

// Tests that Startup and Shutdown do reasonable things.
TEST(SimulatedEventLoopTest, MultinodePingPongStartup) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("aos/events/multinode_pingpong_test_split_config.json"));

  size_t pi1_shutdown_counter = 0;
  size_t pi2_shutdown_counter = 0;
  MessageCounter<examples::Pong> *pi1_pong_counter = nullptr;
  MessageCounter<examples::Ping> *pi2_ping_counter = nullptr;

  message_bridge::TestingTimeConverter time(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory factory(&config.message());
  factory.SetTimeConverter(&time);
  time.AddNextTimestamp(
      distributed_clock::epoch(),
      {BootTimestamp::epoch(), BootTimestamp::epoch(), BootTimestamp::epoch()});

  const chrono::nanoseconds dt = chrono::seconds(10) + chrono::milliseconds(6);

  time.AddNextTimestamp(
      distributed_clock::epoch() + dt,
      {BootTimestamp{.boot = 1, .time = monotonic_clock::epoch()},
       BootTimestamp{.boot = 1, .time = monotonic_clock::epoch()},
       BootTimestamp::epoch() + dt});

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");
  NodeEventLoopFactory *pi2 = factory.GetNodeEventLoopFactory("pi2");

  // Configure startup to start Ping and Pong, and count.
  size_t pi1_startup_counter = 0;
  size_t pi2_startup_counter = 0;
  pi1->OnStartup([pi1]() {
    LOG(INFO) << "Made ping";
    pi1->AlwaysStart<Ping>("ping");
  });
  pi1->OnStartup([&pi1_startup_counter]() { ++pi1_startup_counter; });
  pi2->OnStartup([pi2]() {
    LOG(INFO) << "Made pong";
    pi2->AlwaysStart<Pong>("pong");
  });
  pi2->OnStartup([&pi2_startup_counter]() { ++pi2_startup_counter; });

  // Shutdown just counts.
  pi1->OnShutdown([&pi1_shutdown_counter]() { ++pi1_shutdown_counter; });
  pi2->OnShutdown([&pi2_shutdown_counter]() { ++pi2_shutdown_counter; });

  // Automatically make counters on startup.
  pi1->OnStartup([&pi1_pong_counter, pi1]() {
    pi1_pong_counter = pi1->AlwaysStart<MessageCounter<examples::Pong>>(
        "pi1_pong_counter", "/test");
  });
  pi1->OnShutdown([&pi1_pong_counter]() { pi1_pong_counter = nullptr; });
  pi2->OnStartup([&pi2_ping_counter, pi2]() {
    pi2_ping_counter = pi2->AlwaysStart<MessageCounter<examples::Ping>>(
        "pi2_ping_counter", "/test");
  });
  pi2->OnShutdown([&pi2_ping_counter]() { pi2_ping_counter = nullptr; });

  EXPECT_EQ(pi2_ping_counter, nullptr);
  EXPECT_EQ(pi1_pong_counter, nullptr);

  EXPECT_EQ(pi1_startup_counter, 0u);
  EXPECT_EQ(pi2_startup_counter, 0u);
  EXPECT_EQ(pi1_shutdown_counter, 0u);
  EXPECT_EQ(pi2_shutdown_counter, 0u);

  factory.RunFor(chrono::seconds(10) + chrono::milliseconds(5));
  EXPECT_EQ(pi1_startup_counter, 1u);
  EXPECT_EQ(pi2_startup_counter, 1u);
  EXPECT_EQ(pi1_shutdown_counter, 0u);
  EXPECT_EQ(pi2_shutdown_counter, 0u);
  EXPECT_EQ(pi2_ping_counter->count(), 1001);
  EXPECT_EQ(pi1_pong_counter->count(), 1001);

  LOG(INFO) << pi1->monotonic_now();
  LOG(INFO) << pi2->monotonic_now();

  factory.RunFor(chrono::seconds(5) + chrono::milliseconds(5));

  EXPECT_EQ(pi1_startup_counter, 2u);
  EXPECT_EQ(pi2_startup_counter, 2u);
  EXPECT_EQ(pi1_shutdown_counter, 1u);
  EXPECT_EQ(pi2_shutdown_counter, 1u);
  EXPECT_EQ(pi2_ping_counter->count(), 501);
  EXPECT_EQ(pi1_pong_counter->count(), 501);
}

// Tests that OnStartup handlers can be added after running and get called, and
// can't be called when running.
TEST(SimulatedEventLoopDeathTest, OnStartupWhileRunning) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("aos/events/multinode_pingpong_test_split_config.json"));

  // Test that we can add startup handlers as long as we aren't running, and
  // they get run when Run gets called again.
  // Test that adding a startup handler when running fails.
  //
  // Test shutdown handlers get called on destruction.
  SimulatedEventLoopFactory factory(&config.message());

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");

  int startup_count0 = 0;
  int startup_count1 = 0;

  pi1->OnStartup([&]() { ++startup_count0; });
  EXPECT_EQ(startup_count0, 0);
  EXPECT_EQ(startup_count1, 0);

  factory.RunFor(chrono::nanoseconds(1));
  EXPECT_EQ(startup_count0, 1);
  EXPECT_EQ(startup_count1, 0);

  pi1->OnStartup([&]() { ++startup_count1; });
  EXPECT_EQ(startup_count0, 1);
  EXPECT_EQ(startup_count1, 0);

  factory.RunFor(chrono::nanoseconds(1));
  EXPECT_EQ(startup_count0, 1);
  EXPECT_EQ(startup_count1, 1);

  std::unique_ptr<EventLoop> loop = pi1->MakeEventLoop("foo");
  loop->OnRun([&]() { pi1->OnStartup([]() {}); });

  EXPECT_DEATH({ factory.RunFor(chrono::nanoseconds(1)); },
               "Can only register OnStartup handlers when not running.");
}

// Tests that OnStartup handlers can be added after running and get called, and
// all the handlers get called on reboot.  Shutdown handlers are tested the same
// way.
TEST(SimulatedEventLoopTest, OnStartupShutdownAllRestarts) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("aos/events/multinode_pingpong_test_split_config.json"));

  int startup_count0 = 0;
  int shutdown_count0 = 0;
  int startup_count1 = 0;
  int shutdown_count1 = 0;

  message_bridge::TestingTimeConverter time(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory factory(&config.message());
  factory.SetTimeConverter(&time);
  time.StartEqual();

  const chrono::nanoseconds dt = chrono::seconds(10);
  time.RebootAt(0, distributed_clock::epoch() + dt);
  time.RebootAt(0, distributed_clock::epoch() + 2 * dt);

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");

  pi1->OnStartup([&]() { ++startup_count0; });
  pi1->OnShutdown([&]() { ++shutdown_count0; });
  EXPECT_EQ(startup_count0, 0);
  EXPECT_EQ(startup_count1, 0);
  EXPECT_EQ(shutdown_count0, 0);
  EXPECT_EQ(shutdown_count1, 0);

  factory.RunFor(chrono::nanoseconds(1));
  EXPECT_EQ(startup_count0, 1);
  EXPECT_EQ(startup_count1, 0);
  EXPECT_EQ(shutdown_count0, 0);
  EXPECT_EQ(shutdown_count1, 0);

  pi1->OnStartup([&]() { ++startup_count1; });
  EXPECT_EQ(startup_count0, 1);
  EXPECT_EQ(startup_count1, 0);
  EXPECT_EQ(shutdown_count0, 0);
  EXPECT_EQ(shutdown_count1, 0);

  factory.RunFor(chrono::nanoseconds(1));
  EXPECT_EQ(startup_count0, 1);
  EXPECT_EQ(startup_count1, 1);
  EXPECT_EQ(shutdown_count0, 0);
  EXPECT_EQ(shutdown_count1, 0);

  factory.RunFor(chrono::seconds(15));

  EXPECT_EQ(startup_count0, 2);
  EXPECT_EQ(startup_count1, 2);
  EXPECT_EQ(shutdown_count0, 1);
  EXPECT_EQ(shutdown_count1, 0);

  pi1->OnShutdown([&]() { ++shutdown_count1; });
  factory.RunFor(chrono::seconds(10));

  EXPECT_EQ(startup_count0, 3);
  EXPECT_EQ(startup_count1, 3);
  EXPECT_EQ(shutdown_count0, 2);
  EXPECT_EQ(shutdown_count1, 1);
}

// Tests that event loops which outlive shutdown crash.
TEST(SimulatedEventLoopDeathTest, EventLoopOutlivesReboot) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("aos/events/multinode_pingpong_test_split_config.json"));

  message_bridge::TestingTimeConverter time(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory factory(&config.message());
  factory.SetTimeConverter(&time);
  time.StartEqual();

  const chrono::nanoseconds dt = chrono::seconds(10);
  time.RebootAt(0, distributed_clock::epoch() + dt);

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");

  std::unique_ptr<EventLoop> loop = pi1->MakeEventLoop("foo");

  EXPECT_DEATH({ factory.RunFor(dt * 2); }, "Event loop");
}

// Tests that messages don't survive a reboot of a node.
TEST(SimulatedEventLoopTest, ChannelClearedOnReboot) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("aos/events/multinode_pingpong_test_split_config.json"));

  message_bridge::TestingTimeConverter time(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory factory(&config.message());
  factory.SetTimeConverter(&time);
  time.StartEqual();

  const chrono::nanoseconds dt = chrono::seconds(10);
  time.RebootAt(0, distributed_clock::epoch() + dt);

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");

  const UUID boot_uuid = pi1->boot_uuid();
  EXPECT_NE(boot_uuid, UUID::Zero());

  {
    ::std::unique_ptr<EventLoop> ping_event_loop = pi1->MakeEventLoop("ping");
    aos::Sender<examples::Ping> test_message_sender =
        ping_event_loop->MakeSender<examples::Ping>("/reliable");
    SendPing(&test_message_sender, 1);
  }

  factory.RunFor(chrono::seconds(5));

  {
    ::std::unique_ptr<EventLoop> ping_event_loop = pi1->MakeEventLoop("ping");
    aos::Fetcher<examples::Ping> fetcher =
        ping_event_loop->MakeFetcher<examples::Ping>("/reliable");
    EXPECT_TRUE(fetcher.Fetch());
  }

  factory.RunFor(chrono::seconds(10));

  {
    ::std::unique_ptr<EventLoop> ping_event_loop = pi1->MakeEventLoop("ping");
    aos::Fetcher<examples::Ping> fetcher =
        ping_event_loop->MakeFetcher<examples::Ping>("/reliable");
    EXPECT_FALSE(fetcher.Fetch());
  }
  EXPECT_NE(boot_uuid, pi1->boot_uuid());
}

// Tests that reliable messages get resent on reboot.
TEST(SimulatedEventLoopTest, ReliableMessageResentOnReboot) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("aos/events/multinode_pingpong_test_split_config.json"));

  message_bridge::TestingTimeConverter time(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory factory(&config.message());
  factory.SetTimeConverter(&time);
  time.StartEqual();

  const chrono::nanoseconds dt = chrono::seconds(1);
  time.RebootAt(1, distributed_clock::epoch() + dt);

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");
  NodeEventLoopFactory *pi2 = factory.GetNodeEventLoopFactory("pi2");

  const UUID pi1_boot_uuid = pi1->boot_uuid();
  const UUID pi2_boot_uuid = pi2->boot_uuid();
  EXPECT_NE(pi1_boot_uuid, UUID::Zero());
  EXPECT_NE(pi2_boot_uuid, UUID::Zero());

  {
    ::std::unique_ptr<EventLoop> ping_event_loop = pi1->MakeEventLoop("ping");
    aos::Sender<examples::Ping> test_message_sender =
        ping_event_loop->MakeSender<examples::Ping>("/reliable");
    SendPing(&test_message_sender, 1);
  }

  factory.RunFor(chrono::milliseconds(500));

  {
    ::std::unique_ptr<EventLoop> ping_event_loop = pi2->MakeEventLoop("pong");
    aos::Fetcher<examples::Ping> fetcher =
        ping_event_loop->MakeFetcher<examples::Ping>("/reliable");
    EXPECT_TRUE(fetcher.Fetch());
  }

  factory.RunFor(chrono::seconds(1));

  {
    ::std::unique_ptr<EventLoop> ping_event_loop = pi2->MakeEventLoop("pong");
    aos::Fetcher<examples::Ping> fetcher =
        ping_event_loop->MakeFetcher<examples::Ping>("/reliable");
    EXPECT_TRUE(fetcher.Fetch());
  }
  EXPECT_NE(pi2_boot_uuid, pi2->boot_uuid());
}

}  // namespace testing
}  // namespace aos
