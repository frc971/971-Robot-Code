#include "aos/events/simulated-event-loop.h"
#include "aos/events/event-loop_param_test.h"
#include "gtest/gtest.h"

namespace aos {
namespace testing {

namespace chrono = ::std::chrono;

class SimulatedEventLoopTestFactory : public EventLoopTestFactory {
 public:
  ::std::unique_ptr<EventLoop> Make() override {
    return event_loop.MakeEventLoop();
  }
 private:
   SimulatedEventLoopFactory event_loop;
};

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
}  // namespace testing
}  // namespace aos
