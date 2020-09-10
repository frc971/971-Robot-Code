#include "aos/events/simulated_event_loop.h"

#include <string_view>

#include "aos/events/event_loop_param_test.h"
#include "aos/events/ping_lib.h"
#include "aos/events/pong_lib.h"
#include "aos/events/test_message_generated.h"
#include "aos/network/message_bridge_client_generated.h"
#include "aos/network/message_bridge_server_generated.h"
#include "aos/network/timestamp_generated.h"
#include "gtest/gtest.h"

namespace aos {
namespace testing {

namespace chrono = ::std::chrono;

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

INSTANTIATE_TEST_CASE_P(SimulatedEventLoopCopyTest, AbstractEventLoopTest,
                        ::testing::Values(std::make_tuple(
                            []() {
                              return new SimulatedEventLoopTestFactory();
                            },
                            ReadMethod::COPY)));

INSTANTIATE_TEST_CASE_P(
    SimulatedEventLoopCopyDeathTest, AbstractEventLoopDeathTest,
    ::testing::Values(
        std::make_tuple([]() { return new SimulatedEventLoopTestFactory(); },
                        ReadMethod::COPY)));

INSTANTIATE_TEST_CASE_P(SimulatedEventLoopPinTest, AbstractEventLoopTest,
                        ::testing::Values(std::make_tuple(
                            []() {
                              return new SimulatedEventLoopTestFactory();
                            },
                            ReadMethod::PIN)));

INSTANTIATE_TEST_CASE_P(
    SimulatedEventLoopPinDeathTest, AbstractEventLoopDeathTest,
    ::testing::Values(
        std::make_tuple([]() { return new SimulatedEventLoopTestFactory(); },
                        ReadMethod::PIN)));

// Test that creating an event and running the scheduler runs the event.
TEST(EventSchedulerTest, ScheduleEvent) {
  int counter = 0;
  EventSchedulerScheduler scheduler_scheduler;
  EventScheduler scheduler;
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
  EventScheduler scheduler;
  scheduler_scheduler.AddEventScheduler(&scheduler);

  auto token = scheduler.Schedule(monotonic_clock::epoch() + chrono::seconds(1),
                                  [&counter]() { counter += 1; });
  scheduler.Deschedule(token);
  scheduler_scheduler.Run();
  EXPECT_EQ(counter, 0);
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

template <typename T>
class MessageCounter {
 public:
  MessageCounter(aos::EventLoop *event_loop, std::string_view name) {
    event_loop->MakeNoArgWatcher<T>(name, [this]() { ++count_; });
  }

  size_t count() const { return count_; }

 private:
  size_t count_ = 0;
};

// Tests that ping and pong work when on 2 different nodes, and the message
// gateway messages are sent out as expected.
TEST(SimulatedEventLoopTest, MultinodePingPong) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          "aos/events/multinode_pingpong_config.json");
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

  // Wait to let timestamp estimation start up before looking for the results.
  simulated_event_loop_factory.RunFor(chrono::milliseconds(500));

  int pi1_server_statistics_count = 0;
  pi1_pong_counter_event_loop->MakeWatcher(
      "/pi1/aos", [&pi1_server_statistics_count](
                      const message_bridge::ServerStatistics &stats) {
        VLOG(1) << "pi1 ServerStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 2u);
        for (const message_bridge::ServerConnection *connection :
             *stats.connections()) {
          EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
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
  pi2_pong_counter_event_loop->MakeWatcher(
      "/pi2/aos", [&pi2_server_statistics_count](
                      const message_bridge::ServerStatistics &stats) {
        VLOG(1) << "pi2 ServerStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 1u);

        const message_bridge::ServerConnection *connection =
            stats.connections()->Get(0);
        EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
        EXPECT_GT(connection->sent_packets(), 50);
        EXPECT_TRUE(connection->has_monotonic_offset());
        EXPECT_EQ(connection->monotonic_offset(), 0);
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
        EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
        EXPECT_GE(connection->sent_packets(), 5);
        EXPECT_TRUE(connection->has_monotonic_offset());
        EXPECT_EQ(connection->monotonic_offset(), 0);
        ++pi3_server_statistics_count;
      });

  int pi1_client_statistics_count = 0;
  pi1_pong_counter_event_loop->MakeWatcher(
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

          EXPECT_TRUE(connection->has_monotonic_offset());
          EXPECT_EQ(connection->monotonic_offset(), 150000);
        }
        ++pi1_client_statistics_count;
      });

  int pi2_client_statistics_count = 0;
  pi2_pong_counter_event_loop->MakeWatcher(
      "/pi2/aos", [&pi2_client_statistics_count](
                      const message_bridge::ClientStatistics &stats) {
        VLOG(1) << "pi2 ClientStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 1u);

        const message_bridge::ClientConnection *connection =
            stats.connections()->Get(0);
        EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
        EXPECT_GT(connection->received_packets(), 50);
        EXPECT_TRUE(connection->has_monotonic_offset());
        EXPECT_EQ(connection->monotonic_offset(), 150000);
        ++pi2_client_statistics_count;
      });

  int pi3_client_statistics_count = 0;
  pi3_pong_counter_event_loop->MakeWatcher(
      "/pi3/aos", [&pi3_client_statistics_count](
                      const message_bridge::ClientStatistics &stats) {
        VLOG(1) << "pi3 ClientStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 1u);

        const message_bridge::ClientConnection *connection =
            stats.connections()->Get(0);
        EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
        EXPECT_GE(connection->received_packets(), 5);
        EXPECT_TRUE(connection->has_monotonic_offset());
        EXPECT_EQ(connection->monotonic_offset(), 150000);
        ++pi3_client_statistics_count;
      });

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

  EXPECT_EQ(pi1_server_statistics_count, 9);
  EXPECT_EQ(pi2_server_statistics_count, 9);
  EXPECT_EQ(pi3_server_statistics_count, 9);

  EXPECT_EQ(pi1_client_statistics_count, 95);
  EXPECT_EQ(pi2_client_statistics_count, 95);
  EXPECT_EQ(pi3_client_statistics_count, 95);
}

// Tests that an offset between nodes can be recovered and shows up in
// ServerStatistics correctly.
TEST(SimulatedEventLoopTest, MultinodePingPongWithOffset) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          "aos/events/multinode_pingpong_config.json");
  const Node *pi1 = configuration::GetNode(&config.message(), "pi1");
  const Node *pi2 = configuration::GetNode(&config.message(), "pi2");
  const Node *pi3 = configuration::GetNode(&config.message(), "pi3");

  SimulatedEventLoopFactory simulated_event_loop_factory(&config.message());
  NodeEventLoopFactory *pi2_factory =
      simulated_event_loop_factory.GetNodeEventLoopFactory(pi2);

  constexpr chrono::milliseconds kOffset{1501};
  pi2_factory->SetDistributedOffset(kOffset, 1.0);

  std::unique_ptr<EventLoop> ping_event_loop =
      simulated_event_loop_factory.MakeEventLoop("ping", pi1);
  Ping ping(ping_event_loop.get());

  std::unique_ptr<EventLoop> pong_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pong", pi2);
  Pong pong(pong_event_loop.get());

  std::unique_ptr<EventLoop> pi2_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi2_pong_counter", pi2);

  std::unique_ptr<EventLoop> pi3_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi3_pong_counter", pi3);

  std::unique_ptr<EventLoop> pi1_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi1_pong_counter", pi1);

  // Wait to let timestamp estimation start up before looking for the results.
  simulated_event_loop_factory.RunFor(chrono::milliseconds(500));

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
        EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
        EXPECT_TRUE(connection->has_monotonic_offset());
        EXPECT_EQ(connection->monotonic_offset(), 0);
        ++pi3_server_statistics_count;
      });

  simulated_event_loop_factory.RunFor(chrono::seconds(10) -
                                      chrono::milliseconds(500) +
                                      chrono::milliseconds(5));

  EXPECT_EQ(pi1_server_statistics_count, 9);
  EXPECT_EQ(pi2_server_statistics_count, 9);
  EXPECT_EQ(pi3_server_statistics_count, 9);
}

// Test that disabling statistics actually disables them.
TEST(SimulatedEventLoopTest, MultinodeWithoutStatistics) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          "aos/events/multinode_pingpong_config.json");
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
}

}  // namespace testing
}  // namespace aos
