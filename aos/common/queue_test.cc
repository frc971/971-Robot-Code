#include <unistd.h>

#include <chrono>
#include <memory>

#include "gtest/gtest.h"

#include "aos/common/die.h"
#include "aos/common/test_queue.q.h"
#include "aos/common/util/thread.h"
#include "aos/testing/test_shm.h"

namespace aos {
namespace common {
namespace testing {

namespace chrono = ::std::chrono;

class QueueTest : public ::testing::Test {
 protected:
  void SetUp() override {
    SetDieTestMode(true);
  }

  ::aos::testing::TestSharedMemory my_shm_;
  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  ::aos::Queue<TestingMessage> my_test_queue;

  QueueTest() : my_test_queue(".aos.common.testing.test_queue") {}
};

class MyThread : public util::Thread {
 public:
  MyThread() : threaded_test_queue(".aos.common.testing.test_queue") {}

  virtual void Run() {
    threaded_test_queue.FetchNextBlocking();
    EXPECT_TRUE(threaded_test_queue->test_bool);
    EXPECT_EQ(0x971, threaded_test_queue->test_int);
  }

  ::aos::Queue<TestingMessage> threaded_test_queue;
 private:
  DISALLOW_COPY_AND_ASSIGN(MyThread);
};


// Tests that we can send a message to another thread and it blocking receives
// it at the correct time.
TEST_F(QueueTest, FetchBlocking) {
  MyThread t;
  t.Start();
  usleep(50000);
  my_test_queue.MakeWithBuilder().test_bool(true).test_int(0x971).Send();
  t.Join();
  EXPECT_LE(t.threaded_test_queue.Age(), chrono::milliseconds(57));
}

// Tests that we can send a message with the message pointer and get it back.
TEST_F(QueueTest, SendMessage) {
  ScopedMessagePtr<TestingMessage> msg = my_test_queue.MakeMessage();
  msg->test_bool = true;
  msg->test_int = 0x971;
  msg.Send();

  ASSERT_TRUE(my_test_queue.FetchLatest());
  EXPECT_TRUE(my_test_queue->test_bool);
  EXPECT_EQ(0x971, my_test_queue->test_int);
}

// Tests that we can send a message with the builder and get it back.
TEST_F(QueueTest, SendWithBuilder) {
  my_test_queue.MakeWithBuilder().test_bool(true).test_int(0x971).Send();

  ASSERT_TRUE(my_test_queue.FetchLatest());
  EXPECT_EQ(true, my_test_queue->test_bool);
  EXPECT_EQ(0x971, my_test_queue->test_int);
  EXPECT_EQ(true, my_test_queue.IsNewerThan(chrono::milliseconds(10000)));
}

// Tests that multiple queue instances don't break each other.
TEST_F(QueueTest, MultipleQueues) {
  my_test_queue.MakeWithBuilder().test_bool(true).test_int(0x971).Send();
  ASSERT_TRUE(my_test_queue.FetchLatest());
  EXPECT_TRUE(my_test_queue.get());

  {
    ::aos::Queue<TestingMessage> my_other_test_queue(
        ".aos.common.testing.queue_name");
    my_other_test_queue.MakeMessage();
    EXPECT_FALSE(my_other_test_queue.FetchLatest());
    EXPECT_FALSE(my_test_queue.FetchLatest());
  }

  EXPECT_TRUE(my_test_queue.get());
}

// Tests that using a queue from multiple threads works correctly.
TEST_F(QueueTest, MultipleThreads) {
  my_test_queue.MakeWithBuilder().test_bool(true).test_int(0x971).Send();
  ASSERT_TRUE(my_test_queue.FetchLatest());
  my_test_queue.MakeWithBuilder().test_bool(true).test_int(0x254).Send();
  EXPECT_EQ(0x971, my_test_queue->test_int);

  ::aos::util::FunctionThread::RunInOtherThread([this]() {
    ASSERT_TRUE(my_test_queue.FetchLatest());
    EXPECT_EQ(0x254, my_test_queue->test_int);
  });
  EXPECT_EQ(0x254, my_test_queue->test_int);
}

// Makes sure that MakeWithBuilder zeros the message initially.
// This might randomly succeed sometimes, but it will fail with asan if it
// doesn't.
TEST_F(QueueTest, BuilderZero) {
  my_test_queue.MakeWithBuilder().Send();

  ASSERT_TRUE(my_test_queue.FetchLatest());
  EXPECT_FALSE(my_test_queue->test_bool);
  EXPECT_EQ(0, my_test_queue->test_int);
}

// Tests that various pointer deref functions at least seem to work.
TEST_F(QueueTest, PointerDeref) {
  my_test_queue.MakeWithBuilder().test_bool(true).test_int(0x971).Send();

  ASSERT_TRUE(my_test_queue.FetchLatest());
  const TestingMessage *msg_ptr = my_test_queue.get();
  ASSERT_NE(static_cast<TestingMessage*>(NULL), msg_ptr);
  EXPECT_EQ(0x971, msg_ptr->test_int);
  EXPECT_EQ(msg_ptr, &(*my_test_queue));
}

// Tests that FetchNext doesn't miss any messages.
TEST_F(QueueTest, FetchNext) {
  for (int i = 0; i < 10; ++i) {
    my_test_queue.MakeWithBuilder().test_bool(true).test_int(i).Send();
  }

  for (int i = 0; i < 10; ++i) {
    ASSERT_TRUE(my_test_queue.FetchNext());
    EXPECT_EQ(i, my_test_queue->test_int);
  }
}

// Tests that FetchLatest skips a missing message.
TEST_F(QueueTest, FetchLatest) {
  my_test_queue.MakeWithBuilder().test_bool(true).test_int(0x254).Send();
  my_test_queue.MakeWithBuilder().test_bool(true).test_int(0x971).Send();

  ASSERT_TRUE(my_test_queue.FetchLatest());
  EXPECT_EQ(0x971, my_test_queue->test_int);
}

// Tests that FetchLatest works with multiple readers.
TEST_F(QueueTest, FetchLatestMultiple) {
  ::aos::Queue<TestingMessage> my_second_test_queue(
      ".aos.common.testing.test_queue");
  my_test_queue.MakeWithBuilder().test_bool(true).test_int(0x254).Send();
  my_test_queue.MakeWithBuilder().test_bool(true).test_int(0x971).Send();

  ASSERT_TRUE(my_test_queue.FetchLatest());
  EXPECT_EQ(0x971, my_test_queue->test_int);
  ASSERT_TRUE(my_second_test_queue.FetchLatest());
  ASSERT_TRUE(my_second_test_queue.get() != NULL);
  EXPECT_EQ(0x971, my_second_test_queue->test_int);
}


// Tests that fetching without a new message returns false.
TEST_F(QueueTest, FetchLatestWithoutMessage) {
  my_test_queue.MakeWithBuilder().test_bool(true).test_int(0x254).Send();
  EXPECT_TRUE(my_test_queue.FetchLatest());
  EXPECT_FALSE(my_test_queue.FetchLatest());
  EXPECT_FALSE(my_test_queue.FetchLatest());
  EXPECT_EQ(0x254, my_test_queue->test_int);
}

// Tests that fetching without a message returns false.
TEST_F(QueueTest, FetchOnFreshQueue) {
  EXPECT_FALSE(my_test_queue.FetchLatest());
  EXPECT_EQ(static_cast<TestingMessage*>(NULL), my_test_queue.get());
}

// Tests that fetch next without a message returns false.
TEST_F(QueueTest, FetchNextOnFreshQueue) {
  EXPECT_FALSE(my_test_queue.FetchNext());
  EXPECT_EQ(static_cast<TestingMessage*>(NULL), my_test_queue.get());
}

// Tests that fetch next without a new message returns false.
TEST_F(QueueTest, FetchNextWithoutMessage) {
  my_test_queue.MakeWithBuilder().test_bool(true).test_int(0x254).Send();
  EXPECT_TRUE(my_test_queue.FetchNext());
  EXPECT_FALSE(my_test_queue.FetchNext());
  EXPECT_NE(static_cast<TestingMessage*>(NULL), my_test_queue.get());
}

// Tests that age makes some sense.
TEST_F(QueueTest, Age) {
  my_test_queue.MakeWithBuilder().test_bool(true).test_int(0x971).Send();

  ASSERT_TRUE(my_test_queue.FetchLatest());
  EXPECT_TRUE(my_test_queue.IsNewerThan(chrono::milliseconds(100)));
  const auto age = my_test_queue.Age();
  EXPECT_GE(chrono::nanoseconds(100000000), age);
}


class GroupTest : public ::testing::Test {
 protected:
  GroupTest()
      : my_test_queuegroup(".aos.common.testing.test_queuegroup",
                           0x20561114,
                           ".aos.common.testing.test_queuegroup.first",
                           ".aos.common.testing.test_queuegroup.second") {}

  // Create a new instance of the test group so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  TwoQueues my_test_queuegroup;

 private:
  ::aos::testing::TestSharedMemory my_shm_;
};

// Tests that the hash gets preserved.
TEST_F(GroupTest, Hash) {
  EXPECT_EQ(static_cast<uint32_t>(0x20561114), my_test_queuegroup.hash());
}

// Tests that the hash works.
TEST_F(GroupTest, RealHash) {
  EXPECT_EQ(static_cast<uint32_t>(0x93596b2f), test_queuegroup.hash());
}

// Tests that name works.
TEST_F(GroupTest, Name) {
  EXPECT_EQ(std::string(".aos.common.testing.test_queuegroup"),
            std::string(my_test_queuegroup.name()));
}


class MessageTest : public ::testing::Test {
 public:
  TestingMessage msg;
};

TEST_F(MessageTest, Zeroing) {
  msg.test_bool = true;
  msg.test_int = 0x254;
  msg.SetTimeToNow();

  msg.Zero();

  EXPECT_FALSE(msg.test_bool);
  EXPECT_EQ(0, msg.test_int);
  EXPECT_EQ(monotonic_clock::min_time, msg.sent_time);
}

TEST_F(MessageTest, Size) {
  EXPECT_EQ(static_cast<size_t>(13), msg.Size());
}

TEST_F(MessageTest, Serialize) {
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

// Tests that Print prints out a message nicely.
TEST_F(MessageTest, Print) {
  char printdata[1024];
  msg.test_bool = true;
  msg.test_int = 2056;
  msg.sent_time = monotonic_clock::time_point(chrono::seconds(971) +
                                              chrono::nanoseconds(254));

  std::string golden("971.000000254s, T, 2056");
  EXPECT_EQ(golden.size(), msg.Print(printdata, sizeof(printdata)));

  EXPECT_EQ(golden, std::string(printdata));
}

// Tests that the hash never changes.  If it changes, then someone broke the
// hash routine or changed the message declaration.  Both changes need to be
// validated by hand.
TEST_F(MessageTest, Hash) {
  EXPECT_EQ(static_cast<uint32_t>(0xc33651ac),
            static_cast<uint32_t>(TestingMessage::kHash));
}

TEST_F(MessageTest, SetNow) {
  msg.SetTimeToNow();
  EXPECT_LE(msg.sent_time - monotonic_clock::now(), chrono::milliseconds(20));
}

// Tests that EqualsNoTime works.
TEST_F(MessageTest, EqualsNoTime) {
  msg.test_bool = true;
  msg.test_int = 971;
  TestingMessage other;
  other.test_int = 971;
  EXPECT_FALSE(other.EqualsNoTime(msg));
  EXPECT_FALSE(msg.EqualsNoTime(other));
  other.test_bool = true;
  EXPECT_TRUE(other.EqualsNoTime(msg));
  EXPECT_TRUE(msg.EqualsNoTime(other));
  msg.SetTimeToNow();
  EXPECT_TRUE(msg.EqualsNoTime(other));
}

}  // namespace testing
}  // namespace common
}  // namespace aos
