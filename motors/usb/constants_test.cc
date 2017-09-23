#include "motors/usb/constants.h"

#include "gtest/gtest.h"

namespace frc971 {
namespace teensy {
namespace testing {

TEST(EndpointBufferStateTest, Filling) {
  EXPECT_TRUE(BufferStateHasEmpty(EndpointBufferState::kBothEmptyEvenFirst));
  EXPECT_EQ(EvenOdd::kEven,
            BufferStateToFill(EndpointBufferState::kBothEmptyEvenFirst));

  EXPECT_TRUE(BufferStateHasEmpty(EndpointBufferState::kBothEmptyOddFirst));
  EXPECT_EQ(EvenOdd::kOdd,
            BufferStateToFill(EndpointBufferState::kBothEmptyOddFirst));

  EXPECT_TRUE(BufferStateHasEmpty(EndpointBufferState::kEvenFull));
  EXPECT_EQ(EvenOdd::kOdd, BufferStateToFill(EndpointBufferState::kEvenFull));

  EXPECT_TRUE(BufferStateHasEmpty(EndpointBufferState::kOddFull));
  EXPECT_EQ(EvenOdd::kEven, BufferStateToFill(EndpointBufferState::kOddFull));

  EXPECT_FALSE(BufferStateHasEmpty(EndpointBufferState::kBothFullEvenFirst));
}

TEST(EndpointBufferStateTest, Emptying) {
  EXPECT_FALSE(BufferStateHasFull(EndpointBufferState::kBothEmptyEvenFirst));

  EXPECT_FALSE(BufferStateHasFull(EndpointBufferState::kBothEmptyOddFirst));

  EXPECT_TRUE(BufferStateHasFull(EndpointBufferState::kEvenFull));
  EXPECT_EQ(EvenOdd::kEven, BufferStateToEmpty(EndpointBufferState::kEvenFull));

  EXPECT_TRUE(BufferStateHasFull(EndpointBufferState::kOddFull));
  EXPECT_EQ(EvenOdd::kOdd, BufferStateToEmpty(EndpointBufferState::kOddFull));

  EXPECT_TRUE(BufferStateHasFull(EndpointBufferState::kBothFullEvenFirst));
  EXPECT_EQ(EvenOdd::kEven,
            BufferStateToEmpty(EndpointBufferState::kBothFullEvenFirst));
}

TEST(EndpointBufferStateTest, Transitions) {
  EXPECT_EQ(EndpointBufferState::kEvenFull,
            BufferStateAfterFill(EndpointBufferState::kBothEmptyEvenFirst));

  EXPECT_EQ(EndpointBufferState::kOddFull,
            BufferStateAfterFill(EndpointBufferState::kBothEmptyOddFirst));

  EXPECT_EQ(EndpointBufferState::kBothFullEvenFirst,
            BufferStateAfterFill(EndpointBufferState::kEvenFull));
  EXPECT_EQ(EndpointBufferState::kBothEmptyOddFirst,
            BufferStateAfterEmpty(EndpointBufferState::kEvenFull));

  EXPECT_EQ(EndpointBufferState::kBothFullOddFirst,
            BufferStateAfterFill(EndpointBufferState::kOddFull));
  EXPECT_EQ(EndpointBufferState::kBothEmptyEvenFirst,
            BufferStateAfterEmpty(EndpointBufferState::kOddFull));

  EXPECT_EQ(EndpointBufferState::kOddFull,
            BufferStateAfterEmpty(EndpointBufferState::kBothFullEvenFirst));
}

}  // namespace testing
}  // namespace teensy
}  // namespace frc971
