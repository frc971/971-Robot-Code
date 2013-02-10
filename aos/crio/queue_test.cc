#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#define __TEST_VXWORKS__
#include "aos/common/test_queue.q.h"

using ::aos::time::Time;

namespace aos {
namespace common {
namespace testing {

class CRIOQueueTest : public ::testing::Test {
 protected:
  // Create a new instance of the test queue that is fresh.
  ::aos::Queue<TestingMessage> my_test_queue;

  CRIOQueueTest() : my_test_queue(".aos.common.testing.test_queue") {}
};

// Tests that we can send a message with the message pointer and get it back.
TEST_F(CRIOQueueTest, SendMessage) {
  ScopedMessagePtr<TestingMessage> msg = my_test_queue.MakeMessage();
  msg->test_bool = true;
  msg->test_int = 0x971;
  msg.Send();

  my_test_queue.FetchLatest();
  EXPECT_TRUE(my_test_queue->test_bool);
  EXPECT_EQ(0x971, my_test_queue->test_int);
}

// Tests that we can send a message with the builder and get it back.
TEST_F(CRIOQueueTest, SendWithBuilder) {
  my_test_queue.MakeWithBuilder().test_bool(true).test_int(0x971).Send();

  my_test_queue.FetchLatest();
  EXPECT_EQ(true, my_test_queue->test_bool);
  EXPECT_EQ(0x971, my_test_queue->test_int);
  EXPECT_EQ(true, my_test_queue.IsNewerThanMS(10000));
}

// Tests that various pointer deref functions at least seem to work.
TEST_F(CRIOQueueTest, PointerDeref) {
  my_test_queue.MakeWithBuilder().test_bool(true).test_int(0x971).Send();

  my_test_queue.FetchLatest();
  const TestingMessage *msg_ptr = my_test_queue.get();
  ASSERT_NE(static_cast<TestingMessage*>(NULL), msg_ptr);
  EXPECT_EQ(0x971, msg_ptr->test_int);
  EXPECT_EQ(msg_ptr, &(*my_test_queue));
}

// Tests that FetchLatest skips a missing message.
TEST_F(CRIOQueueTest, FetchLatest) {
  my_test_queue.MakeWithBuilder().test_bool(true).test_int(0x254).Send();
  my_test_queue.MakeWithBuilder().test_bool(true).test_int(0x971).Send();

  my_test_queue.FetchLatest();
  EXPECT_EQ(0x971, my_test_queue->test_int);
}

// Tests that age makes some sense.
TEST_F(CRIOQueueTest, Age) {
  my_test_queue.MakeWithBuilder().test_bool(true).test_int(0x971).Send();

  my_test_queue.FetchLatest();
  EXPECT_TRUE(my_test_queue.IsNewerThanMS(100));
  const Time age = my_test_queue.Age();
  EXPECT_EQ(0, age.sec());
  EXPECT_GE(100000000, age.nsec());
}


class CRIOMessageTest : public ::testing::Test {
 public:
  TestingMessage msg;
};

TEST_F(CRIOMessageTest, Zeroing) {
  msg.test_bool = true;
  msg.test_int = 0x254;
  msg.SetTimeToNow();

  msg.Zero();

  EXPECT_FALSE(msg.test_bool);
  EXPECT_EQ(0, msg.test_int);
  EXPECT_EQ(0, msg.sent_time.sec());
  EXPECT_EQ(0, msg.sent_time.nsec());
}

TEST_F(CRIOMessageTest, Size) {
  EXPECT_EQ(static_cast<size_t>(13), msg.Size());
}

TEST_F(CRIOMessageTest, Serialize) {
  char serialized_data[msg.Size()];
  msg.test_bool = true;
  msg.test_int = 0x254;
  msg.SetTimeToNow();

  msg.Serialize(serialized_data);

  TestingMessage new_msg;
  new_msg.Deserialize(serialized_data);

  EXPECT_EQ(msg.test_bool, new_msg.test_bool);
  EXPECT_EQ(msg.test_int, new_msg.test_int);
  EXPECT_EQ(msg.sent_time, new_msg.sent_time);
}

TEST_F(CRIOMessageTest, SetNow) {
  msg.SetTimeToNow();
  EXPECT_LE(msg.sent_time - Time::Now(), Time::InMS(20));
}

}  // namespace testing
}  // namespace common
}  // namespace aos
