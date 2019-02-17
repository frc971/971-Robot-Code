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

// Tests that using PopSpan with single characters works correctly.
TEST(UartBufferTest, PopSpanSingle) {
  UartBuffer<3> buffer;
  ASSERT_FALSE(buffer.full());
  buffer.PushSingle(1);
  ASSERT_FALSE(buffer.full());
  buffer.PushSingle(2);
  ASSERT_FALSE(buffer.full());

  {
    const auto result = buffer.PopSpan(1);
    ASSERT_EQ(1u, result.size());
    EXPECT_EQ(1u, result[0]);
  }

  ASSERT_FALSE(buffer.full());
  buffer.PushSingle(3);
  ASSERT_FALSE(buffer.full());
  buffer.PushSingle(4);
  ASSERT_TRUE(buffer.full());

  {
    const auto result = buffer.PopSpan(1);
    ASSERT_EQ(1u, result.size());
    EXPECT_EQ(2u, result[0]);
  }

  {
    const auto result = buffer.PopSpan(1);
    ASSERT_EQ(1u, result.size());
    EXPECT_EQ(3u, result[0]);
  }

  {
    const auto result = buffer.PopSpan(1);
    ASSERT_EQ(1u, result.size());
    EXPECT_EQ(4u, result[0]);
  }
}

// Tests that using PopSpan with multiple characters works correctly.
TEST(UartBufferTest, PopSpanMultiple) {
  UartBuffer<1024> buffer;
  for (int i = 0; i < 10; ++i) {
    buffer.PushSingle(i);
  }
  ASSERT_TRUE(buffer.PopSpan(0).empty());
  {
    const auto result = buffer.PopSpan(5);
    ASSERT_EQ(5u, result.size());
    for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(static_cast<char>(i), result[i]);
    }
  }
  {
    const auto result = buffer.PopSpan(10);
    ASSERT_EQ(5u, result.size());
    for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(static_cast<char>(i + 5), result[i]);
    }
  }
  ASSERT_TRUE(buffer.PopSpan(5).empty());
  ASSERT_TRUE(buffer.PopSpan(3000).empty());
  ASSERT_TRUE(buffer.PopSpan(0).empty());
}

// Tests that using PopSpan with multiple characters works correctly when
// wrapping.
TEST(UartBufferTest, PopSpanWrapMultiple) {
  UartBuffer<10> buffer;
  for (int i = 0; i < 10; ++i) {
    buffer.PushSingle(i);
  }
  ASSERT_TRUE(buffer.PopSpan(0).empty());
  {
    const auto result = buffer.PopSpan(5);
    ASSERT_EQ(5u, result.size());
    for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(static_cast<char>(i), result[i]);
    }
  }
  for (int i = 0; i < 5; ++i) {
    buffer.PushSingle(20 + i);
  }
  {
    const auto result = buffer.PopSpan(10);
    ASSERT_EQ(5u, result.size());
    for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(static_cast<char>(i + 5), result[i]);
    }
  }
  {
    const auto result = buffer.PopSpan(10);
    ASSERT_EQ(5u, result.size());
    for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(static_cast<char>(i + 20), result[i]);
    }
  }
}

}  // namespace testing
}  // namespace teensy
}  // namespace frc971
