#include "aos/containers/priority_queue.h"

#include "gtest/gtest.h"

namespace aos {
namespace testing {

// Effectively copies the implementation of ::std::less just to demonstrate how
// things work.
class ExampleCompare {
 public:
  constexpr bool operator()(const int &lhs, const int &rhs) const {
    return lhs < rhs;
  }
};

class PriorityQueueTest : public ::testing::Test {
 public:
  PriorityQueueTest() {}
 protected:
  PriorityQueue<int, 10, ExampleCompare> queue_;
};

TEST_F(PriorityQueueTest, DefaultIsEmpty) {
  ASSERT_EQ(0u, queue_.size());
  ASSERT_TRUE(queue_.empty());
  ASSERT_FALSE(queue_.full());
}

TEST_F(PriorityQueueTest, CanAddData) {
  auto it = queue_.PushFromBottom(5);
  ASSERT_EQ(1u, queue_.size());
  ASSERT_FALSE(queue_.empty());
  ASSERT_FALSE(queue_.full());
  EXPECT_EQ(5, *it);
  EXPECT_EQ(5, queue_.get(0));
  EXPECT_EQ(5, queue_.top());
  EXPECT_EQ(queue_.begin(), it);
  // Also check pre/post-fix increment/decrement operators
  EXPECT_EQ(queue_.end(), ++it);
  EXPECT_EQ(queue_.begin(), --it);
  EXPECT_EQ(queue_.begin(), it++);
  EXPECT_EQ(queue_.end(), it--);
  EXPECT_EQ(queue_.begin(), it);
}

TEST_F(PriorityQueueTest, PriorityInsertion) {
  queue_.PushFromBottom(10);
  queue_.PushFromBottom(20);
  queue_.PushFromBottom(15);
  auto it = queue_.PushFromBottom(11);
  ASSERT_EQ(4u, queue_.size());
  ASSERT_FALSE(queue_.full());
  ::std::vector<int> reverse_expected{20, 15, 11};
  EXPECT_EQ(10, *queue_.begin());
  for (; it != queue_.end(); ++it) {
    EXPECT_EQ(reverse_expected.back(), *it);
    reverse_expected.pop_back();
  }
  ASSERT_TRUE(reverse_expected.empty());
}

// Tests that the clear() method works properly.
TEST_F(PriorityQueueTest, ClearQueue) {
  queue_.PushFromBottom(10);
  queue_.PushFromBottom(20);
  queue_.PushFromBottom(15);
  queue_.PushFromBottom(11);
  ASSERT_EQ(4u, queue_.size());
  ASSERT_FALSE(queue_.full());
  queue_.clear();
  ASSERT_TRUE(queue_.empty());
  ASSERT_EQ(0u, queue_.size());

  queue_.PushFromBottom(1);
  queue_.PushFromBottom(3);
  queue_.PushFromBottom(2);
  ASSERT_EQ(3u, queue_.size());
  ::std::vector<int> reverse_expected{3, 2, 1};
  for (const int val : queue_) {
    EXPECT_EQ(reverse_expected.back(), val);
    reverse_expected.pop_back();
  }
  ASSERT_TRUE(reverse_expected.empty());
}

TEST_F(PriorityQueueTest, FullBufferInsertTop) {
  for (int ii = 0; ii < 10; ++ii) {
    queue_.PushFromBottom(ii);
  }
  ASSERT_EQ(10u, queue_.size());
  ASSERT_TRUE(queue_.full());
  // Check adding value at top.
  queue_.PushFromBottom(100);
  ASSERT_EQ(10u, queue_.size());
  ASSERT_TRUE(queue_.full());
  ::std::vector<int> reverse_expected{100, 9, 8, 7, 6, 5, 4, 3, 2, 1};
  EXPECT_EQ(100, queue_.top());
  for (int val : queue_) {
    EXPECT_EQ(reverse_expected.back(), val);
    reverse_expected.pop_back();
  }
  ASSERT_TRUE(reverse_expected.empty());
}

TEST_F(PriorityQueueTest, FullBufferInsertMiddle) {
  for (int ii = 9; ii >= 0; --ii) {
    queue_.PushFromBottom(ii);
  }
  // Check adding value in the middle.
  queue_.PushFromBottom(5);

  ::std::vector<int> reverse_expected{9, 8, 7, 6, 5, 5, 4, 3, 2, 1};
  EXPECT_EQ(9, queue_.top());
  for (int val : queue_) {
    EXPECT_EQ(reverse_expected.back(), val);
    reverse_expected.pop_back();
  }
  ASSERT_TRUE(reverse_expected.empty());
}

TEST_F(PriorityQueueTest, FullBufferInsertBelowBottom) {
  for (int ii = 9; ii >= 0; --ii) {
    queue_.PushFromBottom(ii);
  }
  // Check adding value at the bottom where it will be dropped.
  queue_.PushFromBottom(-1);

  ::std::vector<int> reverse_expected{9, 8, 7, 6, 5, 4, 3, 2, 1, 0};
  EXPECT_EQ(9, queue_.top());
  for (int val : queue_) {
    EXPECT_EQ(reverse_expected.back(), val);
    reverse_expected.pop_back();
  }
  ASSERT_TRUE(reverse_expected.empty());
}

TEST_F(PriorityQueueTest, FullBufferInsertBottom) {
  for (int ii = 18; ii >= 0; ii -= 2) {
    queue_.PushFromBottom(ii);
  }
  ASSERT_TRUE(queue_.full());
  // Check adding value at the bottom where it displaces the bottom value.
  queue_.PushFromBottom(1);

  ::std::vector<int> reverse_expected{18, 16, 14, 12, 10, 8, 6, 4, 2, 1};
  EXPECT_EQ(18, queue_.top());
  for (int val : queue_) {
    EXPECT_EQ(reverse_expected.back(), val);
    reverse_expected.pop_back();
  }
  ASSERT_TRUE(reverse_expected.empty());
}

// Check that operator-> works as expected on the iterator.
struct TestStruct {
  int a;
  friend bool operator<(const TestStruct &lhs, const TestStruct &rhs) {
    return lhs.a < rhs.a;
  }
};
TEST(PriorirtyQueueTest, MemberAccess) {
  PriorityQueue<TestStruct, 10, ::std::less<TestStruct>> q;
  auto it = q.PushFromBottom({11});
  EXPECT_EQ(11, it->a);
}

}  // namespace testing
}  // namespace aos
