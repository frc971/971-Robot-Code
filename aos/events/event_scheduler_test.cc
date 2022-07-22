#include "aos/events/event_scheduler.h"

#include <chrono>

#include "aos/network/testing_time_converter.h"
#include "gtest/gtest.h"

namespace aos {

namespace chrono = std::chrono;
using aos::logger::BootTimestamp;

// Legacy time converter for keeping old tests working.  Has numerical precision
// problems.
class SlopeOffsetTimeConverter final : public TimeConverter {
 public:
  SlopeOffsetTimeConverter(size_t nodes_count)
      : distributed_offset_(nodes_count, std::chrono::seconds(0)),
        distributed_slope_(nodes_count, 1.0) {
    uuids_.reserve(nodes_count);
    while (uuids_.size() < nodes_count) {
      uuids_.emplace_back(UUID::Random());
    }
  }

  // Sets the offset between the distributed and monotonic clock.
  //   monotonic = distributed * slope + offset;
  void SetDistributedOffset(size_t node_index,
                            std::chrono::nanoseconds distributed_offset,
                            double distributed_slope) {
    distributed_offset_[node_index] = distributed_offset;
    distributed_slope_[node_index] = distributed_slope;
  }

  distributed_clock::time_point ToDistributedClock(
      size_t node_index, BootTimestamp time) override {
    CHECK_EQ(time.boot, 0u);
    return distributed_clock::epoch() +
           std::chrono::duration_cast<std::chrono::nanoseconds>(
               (time.time_since_epoch() - distributed_offset_[node_index]) /
               distributed_slope_[node_index]);
  }

  BootTimestamp FromDistributedClock(size_t node_index,
                                     distributed_clock::time_point time,
                                     size_t boot_index) override {
    CHECK_EQ(boot_index, 0u);
    return {
        .boot = 0u,
        .time = monotonic_clock::epoch() +
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    time.time_since_epoch() * distributed_slope_[node_index]) +
                distributed_offset_[node_index]};
  }

  UUID boot_uuid(size_t node_index, size_t boot_count) override {
    CHECK_EQ(boot_count, 0u);
    return uuids_[node_index];
  }

  void ObserveTimePassed(distributed_clock::time_point /*time*/) override {}

 private:
  // Offset to the distributed clock.
  //   distributed = monotonic + offset;
  std::vector<std::chrono::nanoseconds> distributed_offset_;
  std::vector<double> distributed_slope_;
  std::vector<UUID> uuids_;
};

class FunctionEvent : public EventScheduler::Event {
 public:
  FunctionEvent(std::function<void()> fn) : fn_(fn) {}

  void Handle() noexcept override { fn_(); }

 private:
  std::function<void()> fn_;
};

// Tests that the default parameters (slope of 1, offest of 0) behave as
// an identity.
TEST(EventSchedulerTest, IdentityTimeConversion) {
  SlopeOffsetTimeConverter time(1);
  EventScheduler s(0);
  s.SetTimeConverter(0u, &time);
  EXPECT_EQ(s.FromDistributedClock(distributed_clock::epoch()),
            BootTimestamp::epoch());

  EXPECT_EQ(
      s.FromDistributedClock(distributed_clock::epoch() + chrono::seconds(1)),
      BootTimestamp::epoch() + chrono::seconds(1));

  EXPECT_EQ(s.ToDistributedClock(monotonic_clock::epoch()),
            distributed_clock::epoch());

  EXPECT_EQ(s.ToDistributedClock(monotonic_clock::epoch() + chrono::seconds(1)),
            distributed_clock::epoch() + chrono::seconds(1));
}

// Tests that a non-unity slope is computed correctly.
TEST(EventSchedulerTest, DoubleTimeConversion) {
  SlopeOffsetTimeConverter time(1);
  EventScheduler s(0);
  s.SetTimeConverter(0u, &time);
  time.SetDistributedOffset(0u, std::chrono::seconds(7), 2.0);

  EXPECT_EQ(s.FromDistributedClock(distributed_clock::epoch()),
            BootTimestamp::epoch() + chrono::seconds(7));

  EXPECT_EQ(
      s.FromDistributedClock(distributed_clock::epoch() + chrono::seconds(1)),
      BootTimestamp::epoch() + chrono::seconds(9));

  EXPECT_EQ(s.ToDistributedClock(monotonic_clock::epoch() + chrono::seconds(7)),
            distributed_clock::epoch());

  EXPECT_EQ(s.ToDistributedClock(monotonic_clock::epoch() + chrono::seconds(9)),
            distributed_clock::epoch() + chrono::seconds(1));
}

// Test that RunUntil() stops at the appointed time and returns correctly.
TEST(EventSchedulerTest, RunUntil) {
  int counter = 0;
  EventSchedulerScheduler scheduler_scheduler;
  EventScheduler scheduler(0);
  scheduler_scheduler.AddEventScheduler(&scheduler);

  FunctionEvent e([&counter]() { counter += 1; });
  FunctionEvent quitter(
      [&scheduler_scheduler]() { scheduler_scheduler.Exit(); });
  scheduler.Schedule(monotonic_clock::epoch() + chrono::seconds(1), &e);
  scheduler.Schedule(monotonic_clock::epoch() + chrono::seconds(3), &quitter);
  scheduler.Schedule(monotonic_clock::epoch() + chrono::seconds(5), &e);
  ASSERT_TRUE(scheduler_scheduler.RunUntil(
      realtime_clock::epoch() + std::chrono::seconds(2), &scheduler,
      []() { return std::chrono::nanoseconds{0}; }));
  EXPECT_EQ(counter, 1);
  ASSERT_FALSE(scheduler_scheduler.RunUntil(
      realtime_clock::epoch() + std::chrono::seconds(4), &scheduler,
      []() { return std::chrono::nanoseconds{0}; }));
  EXPECT_EQ(counter, 1);
  ASSERT_TRUE(scheduler_scheduler.RunUntil(
      realtime_clock::epoch() + std::chrono::seconds(6), &scheduler,
      []() { return std::chrono::nanoseconds{0}; }));
  EXPECT_EQ(counter, 2);
}

enum class RunMode {
  kRun,
  kRunUntil,
  kRunFor,
};

// Sets up a parameterized test case that will excercise all three of the Run(),
// RunFor(), and RunUntil() methods of the EventSchedulerScheduler. This exposes
// a ParamRunFor() to the test case that will nominally run for the specified
// time (except for when in kRun mode, where it will just call Run()).
class EventSchedulerParamTest : public testing::TestWithParam<RunMode> {
 public:
  EventSchedulerParamTest() {
    schedulers_.reserve(kNumNodes);
    for (size_t ii = 0; ii < kNumNodes; ++ii) {
      schedulers_.emplace_back(ii);
      schedulers_.back().SetTimeConverter(ii, &time_);
      scheduler_scheduler_.AddEventScheduler(&schedulers_.back());
    }
    scheduler_scheduler_.SetTimeConverter(&time_);
  }

  void StartClocksAtEpoch() {
    time_.AddMonotonic({BootTimestamp::epoch(), BootTimestamp::epoch()});
  }

 protected:
  static constexpr size_t kNumNodes = 2;

  void CheckSchedulersRunning(bool running) {
    for (EventScheduler &scheduler : schedulers_) {
      EXPECT_EQ(running, scheduler.is_running());
    }
  }

  void ParamRunFor(std::chrono::nanoseconds t) {
    switch (GetParam()) {
      case RunMode::kRun:
        scheduler_scheduler_.Run();
        break;
      case RunMode::kRunUntil:
        scheduler_scheduler_.RunUntil(
            realtime_clock::time_point(
                schedulers_.at(0).monotonic_now().time_since_epoch() + t),
            &schedulers_.at(0), []() { return std::chrono::nanoseconds(0); });
        break;
      case RunMode::kRunFor:
        scheduler_scheduler_.RunFor(t);
        break;
    }
  }

  message_bridge::TestingTimeConverter time_{kNumNodes};
  std::vector<EventScheduler> schedulers_;
  EventSchedulerScheduler scheduler_scheduler_;
};

// Tests that we correctly handle exiting during startup.
TEST_P(EventSchedulerParamTest, ExitOnStartup) {
  StartClocksAtEpoch();
  bool observed_handler = false;
  schedulers_.at(0).ScheduleOnStartup([this, &observed_handler]() {
    EXPECT_FALSE(schedulers_.at(0).is_running());
    observed_handler = true;
    scheduler_scheduler_.Exit();
  });
  ParamRunFor(std::chrono::seconds(1));
  EXPECT_TRUE(observed_handler);
}

// Test that creating an event and running the scheduler runs the event.
TEST_P(EventSchedulerParamTest, ScheduleEvent) {
  StartClocksAtEpoch();
  int counter = 0;

  FunctionEvent e([&counter]() { counter += 1; });
  schedulers_.at(0).Schedule(monotonic_clock::epoch() + chrono::seconds(1), &e);
  ParamRunFor(std::chrono::seconds(1));
  EXPECT_EQ(counter, 1);
  auto token = schedulers_.at(0).Schedule(
      monotonic_clock::epoch() + chrono::seconds(2), &e);
  schedulers_.at(0).Deschedule(token);
  ParamRunFor(std::chrono::seconds(2));
  EXPECT_EQ(counter, 1);
}

// Tests that a node that would have a negative monotonic time at boot does not
// get started until later.
TEST_P(EventSchedulerParamTest, NodeWaitsTillEpochToBoot) {
  time_.AddNextTimestamp(
      distributed_clock::epoch(),
      {BootTimestamp{0, monotonic_clock::epoch()},
       BootTimestamp{0, monotonic_clock::epoch() - chrono::seconds(1)}});
  bool observed_startup_0 = false;
  bool observed_startup_1 = false;
  bool observed_on_run_1 = false;
  schedulers_.at(0).ScheduleOnStartup([this, &observed_startup_0]() {
    observed_startup_0 = true;
    EXPECT_FALSE(schedulers_.at(0).is_running());
    EXPECT_FALSE(schedulers_.at(1).is_running());
    EXPECT_EQ(distributed_clock::epoch(),
              scheduler_scheduler_.distributed_now());
    EXPECT_EQ(monotonic_clock::epoch(), schedulers_.at(0).monotonic_now());
    EXPECT_EQ(monotonic_clock::epoch() - chrono::seconds(1),
              schedulers_.at(1).monotonic_now());
  });
  schedulers_.at(1).ScheduleOnStartup([this, &observed_startup_1]() {
    observed_startup_1 = true;
    // Note that we do not *stop* execution on node zero just to get 1 started.
    EXPECT_TRUE(schedulers_.at(0).is_running());
    EXPECT_FALSE(schedulers_.at(1).is_running());
    EXPECT_EQ(distributed_clock::epoch() + chrono::seconds(1),
              scheduler_scheduler_.distributed_now());
    EXPECT_EQ(monotonic_clock::epoch() + chrono::seconds(1),
              schedulers_.at(0).monotonic_now());
    EXPECT_EQ(monotonic_clock::epoch(), schedulers_.at(1).monotonic_now());
  });
  schedulers_.at(1).ScheduleOnRun([this, &observed_on_run_1]() {
    observed_on_run_1 = true;
    // Note that we do not *stop* execution on node zero just to get 1 started.
    EXPECT_TRUE(schedulers_.at(0).is_running());
    EXPECT_TRUE(schedulers_.at(1).is_running());
    EXPECT_EQ(distributed_clock::epoch() + chrono::seconds(1),
              scheduler_scheduler_.distributed_now());
    EXPECT_EQ(monotonic_clock::epoch() + chrono::seconds(1),
              schedulers_.at(0).monotonic_now());
    EXPECT_EQ(monotonic_clock::epoch(), schedulers_.at(1).monotonic_now());
  });

  FunctionEvent e([]() {});
  schedulers_.at(0).Schedule(monotonic_clock::epoch() + chrono::seconds(1), &e);
  ParamRunFor(chrono::seconds(1));
  EXPECT_TRUE(observed_startup_0);
  EXPECT_TRUE(observed_startup_1);
  EXPECT_TRUE(observed_on_run_1);
}

// Tests that a node that never boots does not get any of its handlers run.
TEST_P(EventSchedulerParamTest, NodeNeverBootsIfAlwaysNegative) {
  time_.AddNextTimestamp(
      distributed_clock::epoch(),
      {BootTimestamp{0, monotonic_clock::epoch()},
       BootTimestamp{0, monotonic_clock::epoch() - chrono::seconds(10)}});
  bool observed_startup_0 = false;
  schedulers_.at(0).ScheduleOnStartup([this, &observed_startup_0]() {
    observed_startup_0 = true;
    EXPECT_FALSE(schedulers_.at(0).is_running());
    EXPECT_FALSE(schedulers_.at(1).is_running());
    EXPECT_EQ(distributed_clock::epoch(),
              scheduler_scheduler_.distributed_now());
    EXPECT_EQ(monotonic_clock::epoch(), schedulers_.at(0).monotonic_now());
    EXPECT_EQ(monotonic_clock::epoch() - chrono::seconds(10),
              schedulers_.at(1).monotonic_now());
  });
  schedulers_.at(1).ScheduleOnStartup(
      []() { FAIL() << "Should never have hit startup handlers for node 1."; });
  schedulers_.at(1).ScheduleOnRun(
      []() { FAIL() << "Should never have hit OnRun handlers for node 1."; });
  schedulers_.at(1).set_stopped(
      []() { FAIL() << "Should never have hit stopped handlers for node 1."; });

  FunctionEvent e([this]() { scheduler_scheduler_.Exit(); });
  schedulers_.at(0).Schedule(monotonic_clock::epoch() + chrono::seconds(1), &e);
  ParamRunFor(chrono::seconds(1));
  EXPECT_TRUE(observed_startup_0);
}

// Checks for regressions in how the startup/shutdown handlers behave.
TEST_P(EventSchedulerParamTest, StartupShutdownHandlers) {
  StartClocksAtEpoch();
  time_.AddNextTimestamp(
      distributed_clock::epoch() + chrono::seconds(3),
      {BootTimestamp{0, monotonic_clock::epoch() + chrono::seconds(3)},
       BootTimestamp{0, monotonic_clock::epoch() + chrono::seconds(3)}});
  time_.RebootAt(0, distributed_clock::epoch() + chrono::seconds(4));
  // Expected behavior:
  // If all handlers get called during a reboot, they should sequence as:
  // * is_running_ = false
  // * stopped()
  // * on_shutdown()
  // * on_startup()
  // * started()
  // * is_running_ = true
  // * OnRun()
  //
  // on_shutdown handlers should not get called at end of execution (e.g., when
  // TemporarilyStopAndRun is called)--only when a node reboots.
  //
  // startup and OnRun handlers get cleared after being called once; these are
  // also the only handlers that can have more than one handler registered.
  //
  // Create counters for all the handlers on the 0 node. Create separate a/b
  // counters for the handlers that can/should get cleared.
  int shutdown_counter = 0;
  int stopped_counter = 0;
  int startup_counter_a = 0;
  int startup_counter_b = 0;
  int started_counter = 0;
  int on_run_counter_a = 0;
  int on_run_counter_b = 0;

  schedulers_.at(1).set_on_shutdown([]() {
    FAIL() << "Should never reach the node 1 shutdown handler, since it never "
              "reboots.";
  });

  auto startup_handler_a = [this, &startup_counter_a]() {
    EXPECT_FALSE(schedulers_.at(0).is_running());
    ++startup_counter_a;
  };

  auto startup_handler_b = [this, &startup_counter_b]() {
    EXPECT_FALSE(schedulers_.at(0).is_running());
    ++startup_counter_b;
  };

  auto on_run_handler_a = [this, &on_run_counter_a]() {
    EXPECT_TRUE(schedulers_.at(0).is_running());
    ++on_run_counter_a;
  };

  auto on_run_handler_b = [this, &on_run_counter_b]() {
    EXPECT_TRUE(schedulers_.at(0).is_running());
    ++on_run_counter_b;
  };

  schedulers_.at(0).set_stopped([this, &stopped_counter]() {
    EXPECT_FALSE(schedulers_.at(0).is_running());
    ++stopped_counter;
  });
  schedulers_.at(0).set_on_shutdown(
      [this, &shutdown_counter, startup_handler_a, on_run_handler_a]() {
        EXPECT_FALSE(schedulers_.at(0).is_running());
        schedulers_.at(0).ScheduleOnStartup(startup_handler_a);
        schedulers_.at(0).ScheduleOnRun(on_run_handler_a);
        ++shutdown_counter;
      });
  schedulers_.at(0).ScheduleOnStartup(startup_handler_a);
  schedulers_.at(0).set_started([this, &started_counter]() {
    EXPECT_FALSE(schedulers_.at(0).is_running());
    ++started_counter;
  });
  schedulers_.at(0).ScheduleOnRun(on_run_handler_a);

  FunctionEvent e([]() {});
  schedulers_.at(0).Schedule(monotonic_clock::epoch() + chrono::seconds(1), &e);
  ParamRunFor(std::chrono::seconds(1));
  EXPECT_EQ(shutdown_counter, 0);
  EXPECT_EQ(stopped_counter, 1);
  EXPECT_EQ(started_counter, 1);
  EXPECT_EQ(startup_counter_a, 1);
  EXPECT_EQ(on_run_counter_a, 1);
  EXPECT_EQ(startup_counter_b, 0);
  EXPECT_EQ(on_run_counter_b, 0);

  // In the middle, execute a TemporarilyStopAndRun. Use it to re-register the
  // startup handlers.
  schedulers_.at(0).ScheduleOnStartup(startup_handler_b);
  schedulers_.at(0).ScheduleOnRun(on_run_handler_b);
  FunctionEvent stop_and_run([this, startup_handler_a, on_run_handler_a]() {
    scheduler_scheduler_.TemporarilyStopAndRun(
        [this, startup_handler_a, on_run_handler_a]() {
          schedulers_.at(0).ScheduleOnStartup(startup_handler_a);
          schedulers_.at(0).ScheduleOnRun(on_run_handler_a);
        });
  });
  schedulers_.at(1).Schedule(monotonic_clock::epoch() + chrono::seconds(2),
                             &stop_and_run);
  ParamRunFor(std::chrono::seconds(1));
  EXPECT_EQ(shutdown_counter, 0);
  EXPECT_EQ(stopped_counter, 3);
  EXPECT_EQ(started_counter, 3);
  EXPECT_EQ(startup_counter_a, 2);
  EXPECT_EQ(on_run_counter_a, 2);
  EXPECT_EQ(startup_counter_b, 1);
  EXPECT_EQ(on_run_counter_b, 1);

  // Next, execute a reboot in the middle of running and confirm that things
  // tally correctly. We do not re-register the startup/on_run handlers before
  // starting here, but do in the shutdown handler, so should see the A handlers
  // increment.
  // We need to schedule at least one event so that the reboot is actually
  // observable (otherwise Run() will just terminate immediately, since there
  // are no scheduled events that could possibly observe the reboot anyways).
  schedulers_.at(1).Schedule(monotonic_clock::epoch() + chrono::seconds(5), &e);
  ParamRunFor(std::chrono::seconds(5));
  EXPECT_EQ(shutdown_counter, 1);
  EXPECT_EQ(stopped_counter, 5);
  EXPECT_EQ(started_counter, 5);
  EXPECT_EQ(startup_counter_a, 3);
  EXPECT_EQ(on_run_counter_a, 3);
  EXPECT_EQ(startup_counter_b, 1);
  EXPECT_EQ(on_run_counter_b, 1);
}

// Test that descheduling an already scheduled event doesn't run the event.
TEST_P(EventSchedulerParamTest, DescheduleEvent) {
  StartClocksAtEpoch();
  int counter = 0;
  FunctionEvent e([&counter]() { counter += 1; });
  auto token = schedulers_.at(0).Schedule(
      monotonic_clock::epoch() + chrono::seconds(1), &e);
  schedulers_.at(0).Deschedule(token);
  ParamRunFor(std::chrono::seconds(2));
  EXPECT_EQ(counter, 0);
}

// Test that TemporarilyStopAndRun respects and preserves running.
TEST_P(EventSchedulerParamTest, TemporarilyStopAndRun) {
  StartClocksAtEpoch();
  int counter = 0;

  scheduler_scheduler_.TemporarilyStopAndRun([this]() {
    SCOPED_TRACE("StopAndRun while stopped.");
    CheckSchedulersRunning(false);
  });
  {
    SCOPED_TRACE("After StopAndRun while stopped.");
    CheckSchedulersRunning(false);
  }

  FunctionEvent e([&]() {
    counter += 1;
    {
      SCOPED_TRACE("Before StopAndRun while running.");
      CheckSchedulersRunning(true);
    }
    scheduler_scheduler_.TemporarilyStopAndRun([&]() {
      SCOPED_TRACE("StopAndRun while running.");
      CheckSchedulersRunning(false);
    });
    {
      SCOPED_TRACE("After StopAndRun while running.");
      CheckSchedulersRunning(true);
    }
  });
  schedulers_.at(0).Schedule(monotonic_clock::epoch() + chrono::seconds(1), &e);
  ParamRunFor(std::chrono::seconds(1));
  EXPECT_EQ(counter, 1);
}

// Test that TemporarilyStopAndRun leaves stopped nodes stopped.
TEST_P(EventSchedulerParamTest, TemporarilyStopAndRunStaggeredStart) {
  time_.AddNextTimestamp(
      distributed_clock::epoch(),
      {BootTimestamp{0, monotonic_clock::epoch()},
       BootTimestamp{0, monotonic_clock::epoch() - chrono::seconds(10)}});
  int counter = 0;

  schedulers_[1].ScheduleOnRun([]() { FAIL(); });
  schedulers_[1].ScheduleOnStartup([]() { FAIL(); });
  schedulers_[1].set_on_shutdown([]() { FAIL(); });
  schedulers_[1].set_started([]() { FAIL(); });
  schedulers_[1].set_stopped([]() { FAIL(); });

  FunctionEvent e([this, &counter]() {
    counter += 1;
    EXPECT_TRUE(schedulers_[0].is_running());
    EXPECT_FALSE(schedulers_[1].is_running());
    scheduler_scheduler_.TemporarilyStopAndRun([&]() {
      SCOPED_TRACE("StopAndRun while running.");
      CheckSchedulersRunning(false);
    });
    EXPECT_TRUE(schedulers_[0].is_running());
    EXPECT_FALSE(schedulers_[1].is_running());
  });
  FunctionEvent exiter([this]() { scheduler_scheduler_.Exit(); });
  schedulers_.at(0).Schedule(monotonic_clock::epoch() + chrono::seconds(1), &e);
  schedulers_.at(0).Schedule(monotonic_clock::epoch() + chrono::seconds(2),
                             &exiter);
  ParamRunFor(std::chrono::seconds(1));
  EXPECT_EQ(counter, 1);
}

INSTANTIATE_TEST_SUITE_P(EventSchedulerParamTest, EventSchedulerParamTest,
                         testing::Values(RunMode::kRun, RunMode::kRunFor));

}  // namespace aos
