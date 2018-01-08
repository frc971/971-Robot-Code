#include "aos/events/event-loop_param_test.h"

namespace aos {
namespace testing {

struct TestMessage : public ::aos::Message {
  enum { kQueueLength = 100, kHash = 0x696c0cdc };
  int msg_value;

  void Zero() { msg_value = 0; }
  static size_t Size() { return 1 + ::aos::Message::Size(); }
  size_t Print(char *buffer, size_t length) const;
  TestMessage() { Zero(); }
};

// Ends the given event loop at the given time from now.
void EndEventLoop(EventLoop *loop, ::std::chrono::milliseconds duration) {
  auto end_timer = loop->AddTimer([loop]() { loop->Exit(); });
  end_timer->Setup(loop->monotonic_now() +
                   ::std::chrono::milliseconds(duration));
}

// Tests that watcher and fetcher can fetch from a sender.
// Also tests that OnRun() works.
TEST_P(AbstractEventLoopTest, Basic) {
  auto loop1 = Make();
  auto loop2 = Make();
  auto loop3 = Make();

  auto sender = loop1->MakeSender<TestMessage>("/test");

  auto fetcher = loop2->MakeFetcher<TestMessage>("/test");

  auto msg = sender.MakeMessage();

  msg->msg_value = 200;

  msg.Send();

  EXPECT_TRUE(fetcher.Fetch());
  ASSERT_FALSE(fetcher.get() == nullptr);
  EXPECT_EQ(fetcher->msg_value, 200);

  bool happened = false;

  loop3->OnRun([&]() { happened = true; });

  loop3->MakeWatcher("/test", [&](const TestMessage &message) {
    EXPECT_EQ(message.msg_value, 200);
    loop3->Exit();
  });

  EXPECT_FALSE(happened);
  loop3->Run();
  EXPECT_TRUE(happened);
}

// Verify that making a fetcher and handler for "/test" dies.
TEST_P(AbstractEventLoopTest, FetcherAndHandler) {
  auto loop = Make();
  auto fetcher = loop->MakeFetcher<TestMessage>("/test");
  EXPECT_DEATH(loop->MakeWatcher("/test", [&](const TestMessage &) {}), "/test");
}

// Verify that making 2 fetchers for "/test" fails.
TEST_P(AbstractEventLoopTest, TwoFetcher) {
  auto loop = Make();
  auto fetcher = loop->MakeFetcher<TestMessage>("/test");
  EXPECT_DEATH(loop->MakeFetcher<TestMessage>("/test"), "/test");
}

// Verify that registering a handler twice for "/test" fails.
TEST_P(AbstractEventLoopTest, TwoHandler) {
  auto loop = Make();
  loop->MakeWatcher("/test", [&](const TestMessage &) {});
  EXPECT_DEATH(loop->MakeWatcher("/test", [&](const TestMessage &) {}), "/test");
}

// Verify that Quit() works when there are multiple watchers.
TEST_P(AbstractEventLoopTest, MultipleFetcherQuit) {
  auto loop = Make();

  auto sender = loop->MakeSender<TestMessage>("/test2");
  {
    auto msg = sender.MakeMessage();
    msg->msg_value = 200;
    msg.Send();
  }

  loop->MakeWatcher("/test1", [&](const TestMessage &) {});
  loop->MakeWatcher("/test2", [&](const TestMessage &message) {
    EXPECT_EQ(message.msg_value, 200);
    loop->Exit();
  });
  loop->Run();
}

// Verify that timer intervals and duration function properly.
TEST_P(AbstractEventLoopTest, TimerIntervalAndDuration) {
  auto loop = Make();
  ::std::vector<::aos::monotonic_clock::time_point> iteration_list;

  auto test_timer = loop->AddTimer([&iteration_list, &loop]() {
    iteration_list.push_back(loop->monotonic_now());
  });

  test_timer->Setup(loop->monotonic_now(), ::std::chrono::milliseconds(20));
  EndEventLoop(loop.get(), ::std::chrono::milliseconds(150));
  // Testing that the timer thread waits for the event loop to start before
  // running
  ::std::this_thread::sleep_for(std::chrono::milliseconds(2));
  loop->Run();

  EXPECT_EQ(iteration_list.size(), 8);
}

// Verify that we can change a timer's parameters during execution.
TEST_P(AbstractEventLoopTest, TimerChangeParameters) {
  auto loop = Make();
  ::std::vector<::aos::monotonic_clock::time_point> iteration_list;

  auto test_timer = loop->AddTimer([&iteration_list, &loop]() {
    iteration_list.push_back(loop->monotonic_now());
  });

  auto modifier_timer = loop->AddTimer([&loop, &test_timer]() {
    test_timer->Setup(loop->monotonic_now(), ::std::chrono::milliseconds(30));
  });


  test_timer->Setup(loop->monotonic_now(), ::std::chrono::milliseconds(20));
  modifier_timer->Setup(loop->monotonic_now() +
                        ::std::chrono::milliseconds(45));
  EndEventLoop(loop.get(), ::std::chrono::milliseconds(150));
  loop->Run();

  EXPECT_EQ(iteration_list.size(), 7);
}

// Verify that we can disable a timer during execution.
TEST_P(AbstractEventLoopTest, TimerDisable) {
  auto loop = Make();
  ::std::vector<::aos::monotonic_clock::time_point> iteration_list;

  auto test_timer = loop->AddTimer([&iteration_list, &loop]() {
    iteration_list.push_back(loop->monotonic_now());
  });

  auto ender_timer = loop->AddTimer([&test_timer]() {
    test_timer->Disable();
  });

  test_timer->Setup(loop->monotonic_now(), ::std::chrono::milliseconds(20));
  ender_timer->Setup(loop->monotonic_now() +
                        ::std::chrono::milliseconds(45));
  EndEventLoop(loop.get(), ::std::chrono::milliseconds(150));
  loop->Run();

  EXPECT_EQ(iteration_list.size(), 3);
}
}  // namespace testing
}  // namespace aos
