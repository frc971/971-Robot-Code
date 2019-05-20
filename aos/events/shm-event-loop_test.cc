#include "aos/events/shm-event-loop.h"

#include "aos/events/event-loop_param_test.h"
#include "aos/testing/test_shm.h"
#include "gtest/gtest.h"

namespace aos {
namespace testing {
namespace {

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

struct TestMessage : public ::aos::Message {
  enum { kQueueLength = 100, kHash = 0x696c0cdc };
  int msg_value;

  void Zero() { msg_value = 0; }
  static size_t Size() { return 1 + ::aos::Message::Size(); }
  size_t Print(char *buffer, size_t length) const;
  TestMessage() { Zero(); }
};

}  // namespace

// Tests that FetchNext behaves correctly when we get two messages in the queue
// but don't consume the first until after the second has been sent.
// This cannot be abstracted to AbstractEventLoopTest because not all
// event loops currently support FetchNext().
TEST(ShmEventLoopTest, FetchNextTest) {
  ::aos::testing::TestSharedMemory my_shm;

  ShmEventLoop send_loop;
  ShmEventLoop fetch_loop;
  auto sender = send_loop.MakeSender<TestMessage>("/test");
  Fetcher<TestMessage> fetcher = fetch_loop.MakeFetcher<TestMessage>("/test");

  {
    auto msg = sender.MakeMessage();
    msg->msg_value = 100;
    ASSERT_TRUE(msg.Send());
  }

  {
    auto msg = sender.MakeMessage();
    msg->msg_value = 200;
    ASSERT_TRUE(msg.Send());
  }

  ASSERT_TRUE(fetcher.FetchNext());
  ASSERT_NE(nullptr, fetcher.get());
  EXPECT_EQ(100, fetcher->msg_value);

  ASSERT_TRUE(fetcher.FetchNext());
  ASSERT_NE(nullptr, fetcher.get());
  EXPECT_EQ(200, fetcher->msg_value);

  // When we run off the end of the queue, expect to still have the old message:
  ASSERT_FALSE(fetcher.FetchNext());
  ASSERT_NE(nullptr, fetcher.get());
  EXPECT_EQ(200, fetcher->msg_value);
}
}  // namespace testing
}  // namespace aos
