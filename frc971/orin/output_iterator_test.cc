#include <random>

#include "gtest/gtest.h"

#include "aos/testing/random_seed.h"
#include "frc971/orin/transform_output_iterator.h"

namespace frc971::apriltag::testing {

struct Mul2 {
  uint64_t operator()(const uint32_t num) const {
    return static_cast<uint64_t>(num) * 2;
  }
};

// Tests that the transform output iterator both transforms and otherwise acts
// like a normal pointer
TEST(TransformOutputIteratorTest, IntArr) {
  std::mt19937 generator(aos::testing::RandomSeed());
  std::uniform_int_distribution<uint32_t> random_uint32(0, UINT32_MAX);

  uint32_t *nums_in = (uint32_t *)malloc(UINT32_WIDTH * 20);
  uint64_t *nums_out = (uint64_t *)malloc(UINT64_WIDTH * 20);
  uint64_t *expected_out = (uint64_t *)malloc(UINT64_WIDTH * 20);

  for (size_t i = 0; i < 20; i++) {
    nums_in[i] = random_uint32(generator);
    expected_out[i] = 2 * static_cast<uint64_t>(nums_in[i]);
  }

  Mul2 convert_op;
  TransformOutputIterator<uint32_t, uint64_t, Mul2> itr(nums_out, convert_op);

  // check indirection, array index, increments, and decrements
  EXPECT_EQ(itr == itr, true);
  EXPECT_EQ(itr != itr, false);
  *itr = *nums_in;          // [0]
  *(++itr) = *(++nums_in);  // [1]
  auto temp = itr;
  auto temp2 = itr++;
  EXPECT_EQ(temp, temp2);  // [2]
  EXPECT_NE(temp, itr);
  EXPECT_NE(temp2, itr);
  nums_in++;        // [2]
  *itr = *nums_in;  // [2]
  auto temp3 = ++itr;
  auto temp4 = itr;
  EXPECT_EQ(temp3, temp4);  // [3]
  EXPECT_EQ(temp3, itr);
  itr--;  // [2]
  auto temp5 = --itr;
  auto temp6 = itr;
  EXPECT_EQ(temp5, temp6);  // [1]
  EXPECT_EQ(temp5, itr);
  nums_in--;  // [1]
  auto temp7 = itr;
  auto temp8 = itr--;
  EXPECT_EQ(temp7, temp8);  // [0]
  EXPECT_NE(temp7, itr);
  EXPECT_NE(temp8, itr);
  nums_in--;  // [0]

  for (size_t i = 3; i < 20; i++) {
    itr[i] = nums_in[i];  // [3] -> [19]
  }

  // check expected out and converted out
  for (size_t i = 0; i < 20; i++) {
    EXPECT_EQ(expected_out[i], nums_out[i]);
  }
}
}  // namespace frc971::apriltag::testing
