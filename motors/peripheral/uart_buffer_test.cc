#include "motors/peripheral/uart_buffer.h"

#include "gtest/gtest.h"

namespace frc971 {
namespace teensy {
namespace testing {

// Tests that using PushSpan with single characters works correctly.
TEST(UartBufferTest, PushSpanSingle) {
  UartBuffer<1024> buffer;
  ASSERT_TRUE(buffer.empty());

  {
    ::std::array<char, 1> data{{1}};
    ASSERT_EQ(1, buffer.PushSpan(data));
  }
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(1, buffer.PopSingle());
  ASSERT_TRUE(buffer.empty());

  {
    ::std::array<char, 1> data{{2}};
    ASSERT_EQ(1, buffer.PushSpan(data));
  }
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(2, buffer.PopSingle());
  ASSERT_TRUE(buffer.empty());
}

// Tests that using PushSpan with multiple characters works correctly.
TEST(UartBufferTest, PushSpanMultiple) {
  UartBuffer<1024> buffer;
  ASSERT_TRUE(buffer.empty());

  {
    ::std::array<char, 4> data{{1, 2, 4, 8}};
    ASSERT_EQ(4, buffer.PushSpan(data));
  }
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(1, buffer.PopSingle());
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(2, buffer.PopSingle());
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(4, buffer.PopSingle());
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(8, buffer.PopSingle());
  ASSERT_TRUE(buffer.empty());
}

// Tests that using PushSpan works correctly when wrapping with a
// multiple-character push.
TEST(UartBufferTest, PushSpanWrapMultiple) {
  UartBuffer<4> buffer;
  ASSERT_TRUE(buffer.empty());

  {
    ::std::array<char, 2> data{{10, 20}};
    ASSERT_EQ(2, buffer.PushSpan(data));
  }
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(10, buffer.PopSingle());
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(20, buffer.PopSingle());
  ASSERT_TRUE(buffer.empty());

  {
    ::std::array<char, 4> data{{1, 2, 4, 8}};
    ASSERT_EQ(4, buffer.PushSpan(data));
  }
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(1, buffer.PopSingle());
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(2, buffer.PopSingle());
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(4, buffer.PopSingle());
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(8, buffer.PopSingle());
  ASSERT_TRUE(buffer.empty());
}

// Tests that using PushSpan works correctly when wrapping with a
// single-character push.
TEST(UartBufferTest, PushSpanWrapSingle) {
  UartBuffer<3> buffer;
  ASSERT_TRUE(buffer.empty());

  {
    ::std::array<char, 3> data{{10, 20, 30}};
    ASSERT_EQ(3, buffer.PushSpan(data));
  }
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(10, buffer.PopSingle());
  ASSERT_FALSE(buffer.empty());

  {
    ::std::array<char, 1> data{{1}};
    ASSERT_EQ(1, buffer.PushSpan(data));
  }

  ASSERT_EQ(20, buffer.PopSingle());
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(30, buffer.PopSingle());
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(1, buffer.PopSingle());
  ASSERT_TRUE(buffer.empty());
}

// Tests that using PushSpan works correctly when overflowing with a
// multiple-character push, where none of it fits.
TEST(UartBufferTest, PushSpanOverflowAllMultiple) {
  UartBuffer<2> buffer;
  ASSERT_TRUE(buffer.empty());

  {
    ::std::array<char, 2> data{{10, 20}};
    ASSERT_EQ(2, buffer.PushSpan(data));
  }

  {
    ::std::array<char, 4> data{{1, 2, 4, 8}};
    ASSERT_EQ(0, buffer.PushSpan(data));
  }

  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(10, buffer.PopSingle());
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(20, buffer.PopSingle());
  ASSERT_TRUE(buffer.empty());
}

// Tests that using PushSpan works correctly when overflowing with a
// multiple-character push, where some of it fits.
TEST(UartBufferTest, PushSpanOverflowSomeMultiple) {
  UartBuffer<4> buffer;
  ASSERT_TRUE(buffer.empty());

  {
    ::std::array<char, 2> data{{10, 20}};
    ASSERT_EQ(2, buffer.PushSpan(data));
  }
  ASSERT_FALSE(buffer.empty());

  {
    ::std::array<char, 4> data{{1, 2, 4, 8}};
    ASSERT_EQ(2, buffer.PushSpan(data));
  }

  ASSERT_EQ(10, buffer.PopSingle());
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(20, buffer.PopSingle());
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(1, buffer.PopSingle());
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(2, buffer.PopSingle());
  ASSERT_TRUE(buffer.empty());
}

// Tests that using PushSpan works correctly when overflowing with a
// single-character push.
TEST(UartBufferTest, PushSpanOverflowSingle) {
  UartBuffer<3> buffer;
  ASSERT_TRUE(buffer.empty());

  {
    ::std::array<char, 3> data{{10, 20, 30}};
    ASSERT_EQ(3, buffer.PushSpan(data));
  }
  ASSERT_FALSE(buffer.empty());

  {
    ::std::array<char, 1> data{{1}};
    ASSERT_EQ(0, buffer.PushSpan(data));
  }

  ASSERT_EQ(10, buffer.PopSingle());
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(20, buffer.PopSingle());
  ASSERT_FALSE(buffer.empty());
  ASSERT_EQ(30, buffer.PopSingle());
  ASSERT_TRUE(buffer.empty());
}

}  // namespace testing
}  // namespace teensy
}  // namespace frc971
