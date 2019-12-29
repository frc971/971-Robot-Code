#include "aos/events/shm_event_loop.h"

#include <string_view>

#include "aos/events/event_loop_param_test.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

#include "aos/events/test_message_generated.h"

DECLARE_string(shm_base);
DECLARE_string(override_hostname);

namespace aos {
namespace testing {
namespace {
namespace chrono = ::std::chrono;

class ShmEventLoopTestFactory : public EventLoopTestFactory {
 public:
  ShmEventLoopTestFactory() {
    // Put all the queue files in ${TEST_TMPDIR} if it is set, otherwise
    // everything will be reusing /dev/shm when sharded.
    char *test_tmpdir = getenv("TEST_TMPDIR");
    if (test_tmpdir != nullptr) {
      FLAGS_shm_base = std::string(test_tmpdir) + "/aos";
    }

    // Clean up anything left there before.
    unlink((FLAGS_shm_base + "/test/aos.TestMessage.v1").c_str());
    unlink((FLAGS_shm_base + "/test1/aos.TestMessage.v1").c_str());
    unlink((FLAGS_shm_base + "/test2/aos.TestMessage.v1").c_str());
    unlink((FLAGS_shm_base + "/test2/aos.TestMessage.v1").c_str());
    unlink((FLAGS_shm_base + "/aos/aos.timing.Report.v1").c_str());
  }

  ~ShmEventLoopTestFactory() { FLAGS_override_hostname = ""; }

  ::std::unique_ptr<EventLoop> Make(std::string_view name) override {
    if (configuration()->has_nodes()) {
      FLAGS_override_hostname = "myhostname";
    }
    ::std::unique_ptr<ShmEventLoop> loop(new ShmEventLoop(configuration()));
    loop->set_name(name);
    return std::move(loop);
  }

  ::std::unique_ptr<EventLoop> MakePrimary(std::string_view name) override {
    if (configuration()->has_nodes()) {
      FLAGS_override_hostname = "myhostname";
    }
    ::std::unique_ptr<ShmEventLoop> loop =
        ::std::unique_ptr<ShmEventLoop>(new ShmEventLoop(configuration()));
    primary_event_loop_ = loop.get();
    loop->set_name(name);
    return std::move(loop);
  }

  void Run() override { CHECK_NOTNULL(primary_event_loop_)->Run(); }

  void Exit() override { CHECK_NOTNULL(primary_event_loop_)->Exit(); }

  void SleepFor(::std::chrono::nanoseconds duration) override {
    ::std::this_thread::sleep_for(duration);
  }

 private:
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

}  // namespace

bool IsRealtime() {
  int scheduler;
  PCHECK((scheduler = sched_getscheduler(0)) != -1);

  LOG(INFO) << "scheduler is " << scheduler;
  return scheduler == SCHED_FIFO || scheduler == SCHED_RR;
}

// Tests that every handler type is realtime and runs.  There are threads
// involved and it's easy to miss one.
TEST(ShmEventLoopTest, AllHandlersAreRealtime) {
  ShmEventLoopTestFactory factory;
  auto loop = factory.MakePrimary("primary");
  auto loop2 = factory.Make("loop2");

  loop->SetRuntimeRealtimePriority(1);

  auto sender = loop2->MakeSender<TestMessage>("/test");

  bool did_onrun = false;
  bool did_timer = false;
  bool did_watcher = false;

  auto timer = loop->AddTimer([&did_timer, &factory]() {
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

    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(200);
    msg.Send(builder.Finish());
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
  auto loop1 = factory.MakePrimary("primary");

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
          CHECK_EQ(count, 1);
        } else {
          CHECK_EQ(count, 3);
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

// TODO(austin): Test that missing a deadline with a timer recovers as expected.

}  // namespace testing
}  // namespace aos
