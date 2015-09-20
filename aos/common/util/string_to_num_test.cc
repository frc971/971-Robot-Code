#include <stdint.h>

#include <string>

#include "gtest/gtest.h"

#include "aos/common/util/string_to_num.h"

namespace aos {
namespace util {
namespace testing {

TEST(StringToNumTest, CorrectNumber) {
  int result;
  ASSERT_TRUE(StringToNumber<int>(::std::string("42"), &result));
  EXPECT_EQ(result, 42);
}

TEST(StringToNumTest, NegativeTest) {
  int result;
  ASSERT_TRUE(StringToNumber<int>(::std::string("-42"), &result));
  EXPECT_EQ(result, -42);
}

TEST(StringToNumTest, NonNumber) {
  int result;
  ASSERT_FALSE(StringToNumber<int>(::std::string("Daniel"), &result));
}

TEST(StringToNumTest, NumberWithText) {
  int result;
  ASSERT_FALSE(StringToNumber<int>(::std::string("42Daniel"), &result));
}

TEST(StringToNumTest, OverflowTest) {
  uint32_t result;
  // 2 << 32 should overflow.
  ASSERT_FALSE(StringToNumber<uint32_t>(::std::string("4294967296"), &result));
}

TEST(StringToNumTest, FloatingPointTest) {
  double result;
  ASSERT_TRUE(StringToNumber<double>(::std::string("3.1415927"), &result));
  EXPECT_EQ(result, 3.1415927);
}

}  // testing
}  // util
}  // aos
