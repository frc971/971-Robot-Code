#include "aos/events/simulated_event_loop.h"

#include <string_view>

#include "aos/events/event_loop_param_test.h"
#include "aos/events/test_message_generated.h"
#include "gtest/gtest.h"

namespace aos {
namespace testing {

namespace chrono = ::std::chrono;

class SimulatedEventLoopTestFactory : public EventLoopTestFactory {
 public:
  ::std::unique_ptr<EventLoop> Make(std::string_view name) override {
    MaybeMake();
    return event_loop_factory_->MakeEventLoop(name);
  }
  ::std::unique_ptr<EventLoop> MakePrimary(std::string_view name) override {
    MaybeMake();
    return event_loop_factory_->MakeEventLoop(name);
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
        event_loop_factory_ = std::make_unique<SimulatedEventLoopFactory>(
            configuration(), my_node());
      } else {
        event_loop_factory_ =
            std::make_unique<SimulatedEventLoopFactory>(configuration());
      }
    }
  }
  std::unique_ptr<SimulatedEventLoopFactory> event_loop_factory_;
};

INSTANTIATE_TEST_CASE_P(SimulatedEventLoopDeathTest, AbstractEventLoopDeathTest,
                        ::testing::Values([]() {
                          return new SimulatedEventLoopTestFactory();
                        }));

INSTANTIATE_TEST_CASE_P(SimulatedEventLoopTest, AbstractEventLoopTest,
                        ::testing::Values([]() {
                          return new SimulatedEventLoopTestFactory();
                        }));

// Test that creating an event and running the scheduler runs the event.
TEST(EventSchedulerTest, ScheduleEvent) {
  int counter = 0;
  EventScheduler scheduler;

  scheduler.Schedule(::aos::monotonic_clock::now(),
                      [&counter]() { counter += 1; });
  scheduler.Run();
  EXPECT_EQ(counter, 1);
  auto token = scheduler.Schedule(::aos::monotonic_clock::now(),
                                   [&counter]() { counter += 1; });
  scheduler.Deschedule(token);
  scheduler.Run();
  EXPECT_EQ(counter, 1);
}

// Test that descheduling an already scheduled event doesn't run the event.
TEST(EventSchedulerTest, DescheduleEvent) {
  int counter = 0;
  EventScheduler scheduler;

  auto token = scheduler.Schedule(::aos::monotonic_clock::now(),
                                   [&counter]() { counter += 1; });
  scheduler.Deschedule(token);
  scheduler.Run();
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
            simulated_event_loop_factory.monotonic_now());
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
            simulated_event_loop_factory.monotonic_now());
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
  VLOG(1) << FlatbufferToJson(primary_report, true);

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

}  // namespace testing
}  // namespace aos
