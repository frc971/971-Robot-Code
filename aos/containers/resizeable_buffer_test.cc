#include "aos/containers/resizeable_buffer.h"

#include <numeric>

#include "absl/types/span.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace aos::testing {

// Tests using resize to do some stuff with a ResizeableBuffer.
TEST(ResizeableBufferTest, Resize) {
  ResizeableBuffer buffer;
  EXPECT_EQ(buffer.size(), 0u);
  EXPECT_EQ(buffer.capacity(), 0u);

  buffer.resize(3);
  EXPECT_EQ(buffer.size(), 3u);
  EXPECT_GE(buffer.capacity(), 3u);
  EXPECT_EQ(buffer.begin() + 3, buffer.end());

  buffer.resize(5);
  EXPECT_EQ(buffer.size(), 5u);
  EXPECT_GE(buffer.capacity(), 5u);
  EXPECT_EQ(buffer.begin() + 5, buffer.end());

  buffer.resize(2);
  EXPECT_EQ(buffer.size(), 2u);
  EXPECT_GE(buffer.capacity(), 2u);
  EXPECT_EQ(buffer.begin() + 2, buffer.end());
}

// Tests using erase_front on a ResizeableBuffer.
TEST(ResizeableBufferTest, EraseFront) {
  ResizeableBuffer buffer;
  EXPECT_EQ(buffer.size(), 0u);
  EXPECT_EQ(buffer.capacity(), 0u);

  buffer.resize(5);
  for (int i = 0; i < 5; ++i) {
    buffer.begin()[i] = '1' + i;
  }
  EXPECT_THAT(absl::Span<uint8_t>(buffer.begin(), buffer.size()),
              ::testing::ElementsAre('1', '2', '3', '4', '5'));
  buffer.erase_front(2);
  EXPECT_THAT(absl::Span<uint8_t>(buffer.begin(), buffer.size()),
              ::testing::ElementsAre('3', '4', '5'));
}

// Tests using push_back starting from an empty ResizeableBuffer.
TEST(ResizeableBufferTest, PushBackEmpty) {
  ResizeableBuffer buffer;
  buffer.push_back('1');
  buffer.push_back('2');
  buffer.push_back('3');
  EXPECT_THAT(absl::Span<uint8_t>(buffer.begin(), buffer.size()),
              ::testing::ElementsAre('1', '2', '3'));
}

// Tests using push_back starting from a non-empty ResizeableBuffer.
TEST(ResizeableBufferTest, PushBackNonEmpty) {
  ResizeableBuffer buffer;
  EXPECT_EQ(buffer.size(), 0u);
  EXPECT_EQ(buffer.capacity(), 0u);

  buffer.resize(2);
  for (int i = 0; i < 2; ++i) {
    buffer.begin()[i] = '1' + i;
  }
  buffer.push_back('3');
  buffer.push_back('4');
  buffer.push_back('5');
  EXPECT_THAT(absl::Span<uint8_t>(buffer.begin(), buffer.size()),
              ::testing::ElementsAre('1', '2', '3', '4', '5'));
}

// Tests using push_back starting from a large non-empty ResizeableBuffer.
TEST(ResizeableBufferTest, PushBackLargeNonEmpty) {
  ResizeableBuffer buffer;
  EXPECT_EQ(buffer.size(), 0u);
  EXPECT_EQ(buffer.capacity(), 0u);

  buffer.resize(27);
  for (int i = 0; i < 27; ++i) {
    buffer.begin()[i] = '1' + i;
  }
  buffer.push_back('1' + 27);
  buffer.push_back('1' + 28);
  std::vector<char> expected(29);
  std::iota(expected.begin(), expected.end(), '1');
  EXPECT_THAT(absl::Span<uint8_t>(buffer.begin(), buffer.size()),
              ::testing::ElementsAreArray(expected));
}

}  // namespace aos::testing
