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

}  // namespace testing
}  // namespace aos
