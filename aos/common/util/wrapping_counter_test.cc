#include "aos/common/util/wrapping_counter.h"

#include <limits.h>

#include "gtest/gtest.h"

namespace aos {
namespace util {
namespace testing {

TEST(WrappingCounterTest, Basic) {
  WrappingCounter test_counter;
  EXPECT_EQ(0, test_counter.count());
  EXPECT_EQ(1, test_counter.Update(1));
  EXPECT_EQ(1, test_counter.Update(1));
  EXPECT_EQ(2, test_counter.Update(2));
  EXPECT_EQ(7, test_counter.Update(7));
  EXPECT_EQ(7, test_counter.count());
  EXPECT_EQ(123, test_counter.Update(123));
  EXPECT_EQ(123, test_counter.count());
}

TEST(WrappingCounterTest, Reset) {
  WrappingCounter test_counter;
  test_counter.Update(5);
  test_counter.Reset();
  EXPECT_EQ(0, test_counter.count());
  test_counter.Reset(56);
  EXPECT_EQ(56, test_counter.count());
}

namespace {
void test_wrapping(int16_t start, int16_t step) {
  WrappingCounter test_counter;
  for (int16_t i = start; i < INT16_MAX - step; i += step) {
    EXPECT_EQ(i, test_counter.Update(i & 0xFF));
  }
}
}

// This tests the basic wrapping functionality.
TEST(WrappingCounterTest, ReasonableWrapping) {
  test_wrapping(0, 13);
  test_wrapping(0, 53);
  test_wrapping(0, 64);
  test_wrapping(0, 73);
}

// It would be reasonable for these to fail if the implementation changes.
TEST(WrappingCounterTest, UnreasonableWrapping) {
  test_wrapping(0, 128);
  test_wrapping(0, 213);
  test_wrapping(0, 255);
}

}  // namespace testing
}  // namespace util
}  // namespace aos
