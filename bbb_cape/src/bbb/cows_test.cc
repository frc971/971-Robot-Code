#include "cape/cows.h"

#include <gtest/gtest.h>

namespace testing {

TEST(CowsTest, StupidZeros) {
  static const uint8_t kTestInput[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x08, 0x01,
                                       0x00};
  uint32_t input[2];
  memcpy(input, kTestInput, 8);
  uint32_t output[2];
  EXPECT_EQ(
      1u, cows_unstuff(input, sizeof(kTestInput), output, sizeof(output) * 4));
}

}  // namespace testing
