#include "aos/events/simulated-event-loop.h"
#include "aos/events/event-loop_param_test.h"
#include "gtest/gtest.h"

namespace aos {
namespace testing {

namespace chrono = ::std::chrono;

class SimulatedEventLoopTestFactory : public EventLoopTestFactory {
 public:
  ::std::unique_ptr<EventLoop> Make() override {
    return event_loop_factory_.MakeEventLoop();
  }
  ::std::unique_ptr<EventLoop> MakePrimary() override {
    return event_loop_factory_.MakeEventLoop();
  }

  void Run() override { event_loop_factory_.Run(); }

  // TODO(austin): Implement this.  It's used currently for a phased loop test.
  // I'm not sure how much that matters.
  void SleepFor(::std::chrono::nanoseconds /*duration*/) override {}

 private:
   SimulatedEventLoopFactory event_loop_factory_;
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
  SimulatedEventLoopFactory simulated_event_loop_factory;
  ::std::unique_ptr<EventLoop> event_loop =
      simulated_event_loop_factory.MakeEventLoop();

  simulated_event_loop_factory.RunFor(chrono::seconds(1));

  EXPECT_EQ(::aos::monotonic_clock::epoch() + chrono::seconds(1),
            simulated_event_loop_factory.monotonic_now());
  EXPECT_EQ(::aos::monotonic_clock::epoch() + chrono::seconds(1),
            event_loop->monotonic_now());
}

// Test that running for a time with a periodic handler causes time to end
// correctly.
TEST(SimulatedEventLoopTest, RunForTimerHandler) {
  SimulatedEventLoopFactory simulated_event_loop_factory;
  ::std::unique_ptr<EventLoop> event_loop =
      simulated_event_loop_factory.MakeEventLoop();

  int counter = 0;
  auto timer = event_loop->AddTimer([&counter, &event_loop]() { ++counter; });
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

}  // namespace testing
}  // namespace aos
