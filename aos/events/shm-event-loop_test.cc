#include "aos/events/shm-event-loop.h"

#include "aos/events/event-loop_param_test.h"
#include "aos/testing/test_shm.h"
#include "gtest/gtest.h"

namespace aos {
namespace testing {
namespace {
namespace chrono = ::std::chrono;

class ShmEventLoopTestFactory : public EventLoopTestFactory {
 public:
  ::std::unique_ptr<EventLoop> Make() override {
    return ::std::unique_ptr<EventLoop>(new ShmEventLoop());
  }

  ::std::unique_ptr<EventLoop> MakePrimary() override {
    ::std::unique_ptr<ShmEventLoop> loop =
        ::std::unique_ptr<ShmEventLoop>(new ShmEventLoop());
    primary_event_loop_ = loop.get();
    return ::std::move(loop);
  }

  void Run() override { CHECK_NOTNULL(primary_event_loop_)->Run(); }

  void Exit() override { CHECK_NOTNULL(primary_event_loop_)->Exit(); }

  void SleepFor(::std::chrono::nanoseconds duration) override {
    ::std::this_thread::sleep_for(duration);
  }

 private:
  ::aos::testing::TestSharedMemory my_shm_;

  ::aos::ShmEventLoop *primary_event_loop_;
};

INSTANTIATE_TEST_CASE_P(ShmEventLoopTest, AbstractEventLoopTest,
                        ::testing::Values([]() {
                          return new ShmEventLoopTestFactory();
                        }));

INSTANTIATE_TEST_CASE_P(ShmEventLoopDeathTest, AbstractEventLoopDeathTest,
                        ::testing::Values([]() {
                          return new ShmEventLoopTestFactory();
                        }));

struct TestMessage : public ::aos::Message {
  enum { kQueueLength = 100, kHash = 0x696c0cdc };
  int msg_value;

  void Zero() { msg_value = 0; }
  static size_t Size() { return 1 + ::aos::Message::Size(); }
  size_t Print(char *buffer, size_t length) const;
  TestMessage() { Zero(); }
};

}  // namespace

bool IsRealtime() {
  int scheduler;
  if ((scheduler = sched_getscheduler(0)) == -1) {
    PLOG(FATAL, "sched_getscheduler(0) failed\n");
  }
  LOG(INFO, "scheduler is %d\n", scheduler);
  return scheduler == SCHED_FIFO || scheduler == SCHED_RR;
}

// Tests that every handler type is realtime and runs.  There are threads
// involved and it's easy to miss one.
TEST(ShmEventLoopTest, AllHandlersAreRealtime) {
  ShmEventLoopTestFactory factory;
  auto loop = factory.MakePrimary();
  auto loop2 = factory.Make();

  loop->SetRuntimeRealtimePriority(1);

  auto sender = loop2->MakeSender<TestMessage>("/test");

  bool did_onrun = false;
  bool did_timer = false;
  bool did_watcher = false;

  auto timer = loop->AddTimer([&did_timer, &loop, &factory]() {
    EXPECT_TRUE(IsRealtime());
    did_timer = true;
    factory.Exit();
  });

  loop->MakeWatcher("/test", [&did_watcher](const TestMessage &) {
    EXPECT_TRUE(IsRealtime());
    did_watcher = true;
  });

  loop->OnRun([&loop, &did_onrun, &sender, timer]() {
    EXPECT_TRUE(IsRealtime());
    did_onrun = true;
    timer->Setup(loop->monotonic_now() + chrono::milliseconds(100));
    auto msg = sender.MakeMessage();
    msg->msg_value = 200;
    msg.Send();
  });

  factory.Run();

  EXPECT_TRUE(did_onrun);
  EXPECT_TRUE(did_timer);
  EXPECT_TRUE(did_watcher);
}

// Tests that missing a deadline inside the function still results in PhasedLoop
// running at the right offset.
TEST(ShmEventLoopTest, DelayedPhasedLoop) {
  ShmEventLoopTestFactory factory;
  auto loop1 = factory.MakePrimary();

  ::std::vector<::aos::monotonic_clock::time_point> times;

  constexpr chrono::milliseconds kOffset = chrono::milliseconds(400);

  loop1->AddPhasedLoop(
      [&times, &loop1, &kOffset, &factory](int count) {
        const ::aos::monotonic_clock::time_point monotonic_now =
            loop1->monotonic_now();

        // Compute our offset.
        const ::aos::monotonic_clock::duration remainder =
            monotonic_now.time_since_epoch() -
            chrono::duration_cast<chrono::seconds>(
                monotonic_now.time_since_epoch());

        // Make sure we we are called near where we should be even when we
        // delay.
        constexpr chrono::milliseconds kEpsilon(200);
        EXPECT_LT(remainder, kOffset + kEpsilon);
        EXPECT_GT(remainder, kOffset - kEpsilon);

        // Confirm that we see the missed count when we sleep.
        if (times.size() == 0) {
          EXPECT_EQ(count, 1);
        } else {
          EXPECT_EQ(count, 3);
        }

        times.push_back(loop1->monotonic_now());
        if (times.size() == 2) {
          factory.Exit();
        }

        // Now, add a large delay.  This should push us up to 3 cycles.
        ::std::this_thread::sleep_for(chrono::milliseconds(2500));
      },
      chrono::seconds(1), kOffset);

  factory.Run();

  EXPECT_EQ(times.size(), 2u);
}

}  // namespace testing
}  // namespace aos
