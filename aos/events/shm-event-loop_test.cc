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

  auto timer = loop->AddTimer([&did_timer, &loop]() {
    EXPECT_TRUE(IsRealtime());
    did_timer = true;
    loop->Exit();
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
}  // namespace testing
}  // namespace aos
