#include "aos/common/ring_buffer.h"

#include "gtest/gtest.h"

namespace aos {
namespace testing {

class RingBufferTest : public ::testing::Test {
 public:
  RingBufferTest() {}

 protected:
  RingBuffer<int, 10> buffer_;
};

// Test if the RingBuffer is empty when initialized properly
TEST_F(RingBufferTest, DefaultIsEmpty) {
  // The RingBuffer should have a size of 0, a capacity of 10 (note that it was
  // initialized as 10), have no items, and not be full
  ASSERT_EQ(0u, buffer_.size());
  ASSERT_EQ(10u, buffer_.capacity());
  ASSERT_TRUE(buffer_.empty());
  ASSERT_FALSE(buffer_.full());
}

// Test that the RingBuffer can fill it's entire capacity and read back the data
TEST_F(RingBufferTest, CanAddData) {
  ASSERT_TRUE(buffer_.empty());

  // Add sequential numbers to the RingBuffer
  // (the value of each item is it's index #)
  for (size_t i = 0; i < buffer_.capacity(); ++i) {
    // The buffer shouldn't be full yet, and it's size should be how many items
    // we've added so far. Once that happens, we add another item
    ASSERT_FALSE(buffer_.full());
    ASSERT_EQ(i, buffer_.size());
    buffer_.Push(i);

    // The buffer shouldn't be empty and it's size should be 1 more since we
    // just
    // added an item. Also, the last item in the buffer should equal the one we
    // just added
    ASSERT_FALSE(buffer_.empty());
    ASSERT_EQ(i + 1, buffer_.size());
    ASSERT_EQ(i, buffer_[i]);
  }

  ASSERT_TRUE(buffer_.full());
}

// Tests that the RingBuffer properly loops back and starts overwriting from the
// first element after being filled up
TEST_F(RingBufferTest, OverfillData) {
  // Add numbers 0-24 to the RingBuffer
  for (int i = 0; i < 25; ++i) {
    buffer_.Push(i);
  }

  // It should now be full
  ASSERT_TRUE(buffer_.full());

  // Since the buffer is a size of 10 and has been filled up 2.5 times, it
  // should
  // now contain the numbers 15-24
  for (size_t i = 0; i < buffer_.size(); ++i) {
    ASSERT_EQ(15 + i, buffer_[i]);
  }
}

// Tests shifting from the front of the ringbuffer.
TEST_F(RingBufferTest, RingBufferShift) {
  // Add numbers 0-24 to the RingBuffer
  for (int i = 0; i < 25; ++i) {
    buffer_.Push(i);
  }

  // It should now be full
  ASSERT_TRUE(buffer_.full());

  buffer_.Shift();
  buffer_.Shift();
  buffer_.Shift();

  ASSERT_EQ(buffer_.size(), 7);

  // The buffer should now contain the numbers 18-24
  for (size_t i = 0; i < buffer_.size(); ++i) {
    ASSERT_EQ(18 + i, buffer_[i]);
  }
}

// Test that the buffer works after Reset.
TEST_F(RingBufferTest, ResetWorks) {
  // Over fill it, and then clear it out.
  ASSERT_TRUE(buffer_.empty());

  for (size_t i = 0; i < 53; ++i) {
    buffer_.Push(i);
  }
  ASSERT_TRUE(buffer_.full());

  buffer_.Reset();

  ASSERT_TRUE(buffer_.empty());

  // Now, add numbers 0-9 to the RingBuffer.
  for (int i = 0; i < 10; ++i) {
    buffer_.Push(i);
  }

  // It should now be full.
  ASSERT_TRUE(buffer_.full());

  // The last 10 numbers were added 0-9, so verify that is what is in the
  // buffer.
  for (size_t i = 0; i < buffer_.size(); ++i) {
    ASSERT_EQ(i, buffer_[i]);
  }
}

}  // namespace testing
}  // namespace aos
