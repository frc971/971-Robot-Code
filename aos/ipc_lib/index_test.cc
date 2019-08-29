#include "aos/ipc_lib/index.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace aos {
namespace ipc_lib {
namespace testing {

class QueueIndexTest : public ::testing::Test {
 protected:
  uint32_t GetIndex(const QueueIndex &index) {
    LOG(INFO) << "Index, count: " << std::hex << index.index_ << ", "
              << index.count_;
    return index.index();
  }

  QueueIndex Make(uint32_t index, uint32_t count) {
    return QueueIndex(index, count);
  }
};

// Tests that an invalid index is invalid.
TEST_F(QueueIndexTest, TestValid) {
  QueueIndex invalid = QueueIndex::Invalid();

  EXPECT_EQ(0xffffffff, GetIndex(invalid));
  EXPECT_FALSE(invalid.valid());

  QueueIndex valid = Make(5, 7);

  EXPECT_TRUE(valid.valid());
}

// Tests that the max index function returns the max index as expected.
TEST_F(QueueIndexTest, TestMaxIndex) {
  EXPECT_EQ(QueueIndex::MaxIndex(12u, 4u), 12u);
  EXPECT_EQ(QueueIndex::MaxIndex(13u, 4u), 12u);
  EXPECT_EQ(QueueIndex::MaxIndex(14u, 4u), 12u);
  EXPECT_EQ(QueueIndex::MaxIndex(15u, 4u), 12u);
  EXPECT_EQ(QueueIndex::MaxIndex(16u, 4u), 16u);
}

// Tests that incrementing an index works as expected.
TEST_F(QueueIndexTest, TestIncrement) {
  QueueIndex zero = Make(0, 3);

  EXPECT_EQ(GetIndex(zero), 0u);

  QueueIndex one = zero.Increment();
  EXPECT_EQ(GetIndex(one), 1u);

  // Try it with a base 3.  Apparently 3 fits exactly.
  {
    QueueIndex two_below = Make(QueueIndex::MaxIndex(0xffffffffu, 3u) - 2, 3);
    EXPECT_EQ(GetIndex(two_below), 0xfffffffdu);

    QueueIndex one_below = two_below.Increment();
    EXPECT_EQ(GetIndex(one_below), 0xfffffffeu);

    QueueIndex wrapped = one_below.Increment();
    EXPECT_EQ(GetIndex(wrapped), 0);

    EXPECT_EQ(wrapped, zero);
  }

  // Now try it with base 4.  Should still work.
  {
    QueueIndex two_below = Make(QueueIndex::MaxIndex(0xffffffffu, 4u) - 2, 4);
    EXPECT_EQ(GetIndex(two_below), 0xfffffffau);

    QueueIndex one_below = two_below.Increment();
    EXPECT_EQ(GetIndex(one_below), 0xfffffffbu);

    QueueIndex wrapped = one_below.Increment();
    EXPECT_EQ(GetIndex(wrapped), 0);
  }
}

// Tests that decrementing and incrementing again an index works as expected.
TEST_F(QueueIndexTest, TestDecrement) {
  {
    QueueIndex zero = Make(0, 3);
    EXPECT_EQ(GetIndex(zero), 0x00000000u);

    QueueIndex negative10 = zero.DecrementBy(10);
    EXPECT_EQ(GetIndex(negative10), 0xffffffff - 10);

    EXPECT_EQ(zero, negative10.IncrementBy(10));
  }
  {
    QueueIndex zero = Make(0, 4);
    EXPECT_EQ(GetIndex(zero), 0x00000000u);

    QueueIndex negative10 = zero.DecrementBy(10);
    EXPECT_EQ(GetIndex(negative10), 0xfffffffc - 10);

    EXPECT_EQ(zero, negative10.IncrementBy(10));
  }

  {
    QueueIndex five = Make(5, 3);
    EXPECT_EQ(GetIndex(five), 5u);

    QueueIndex negative10 = five.DecrementBy(10);
    EXPECT_EQ(GetIndex(negative10), 0xffffffff - 5);

    EXPECT_EQ(five, negative10.IncrementBy(10));
  }
  {
    QueueIndex five = Make(5, 4);
    EXPECT_EQ(GetIndex(five), 5u);

    QueueIndex negative10 = five.DecrementBy(10);
    EXPECT_EQ(GetIndex(negative10), 0xfffffffc - 5);

    EXPECT_EQ(five, negative10.IncrementBy(10));
  }
}

// Tests that an invalid index is invalid, and a valid one is valid.
TEST(IndexTest, TestInvalid) {
  EXPECT_FALSE(Index::Invalid().valid());

  EXPECT_TRUE(Index(0, 0).valid());
}

// Tests that we get back the two indices as expected.
TEST(IndexTest, TestRecoverIndices) {
  QueueIndex five = QueueIndex::Zero(100).IncrementBy(5);
  Index index(five, 11);
  EXPECT_EQ(index.queue_index(), 5);
  EXPECT_EQ(index.message_index(), 11);
}

// Tests that Plausible behaves.
TEST(IndexTest, TestPlausible) {
  QueueIndex five = QueueIndex::Zero(100).IncrementBy(5);
  QueueIndex ffff = QueueIndex::Zero(100).IncrementBy(0xffff);

  // Tests that if five has wrapped, we still return plausible.
  for (int i = 0; i < 100; ++i) {
    Index index(five, i);
    EXPECT_EQ(index.queue_index(), 5);

    EXPECT_TRUE(index.IsPlausible(five));

    EXPECT_EQ(index.message_index(), i);

    five = five.IncrementBy(0x10000);
  }

  // Tests that a queue index with a value of 0xffff doesn't match an invalid
  // index.
  for (int i = 0; i < 100; ++i) {
    Index index(ffff, i);
    EXPECT_EQ(index.queue_index(), 0xffff);

    EXPECT_TRUE(index.IsPlausible(ffff));
    EXPECT_FALSE(index.IsPlausible(QueueIndex::Invalid()));

    EXPECT_EQ(index.message_index(), i);
  }
}

}  // namespace testing
}  // namespace ipc_lib
}  // namespace aos
