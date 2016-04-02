#include "gtest/gtest.h"
#include "frc971/zeroing/averager.h"

namespace frc971 {
namespace zeroing {

class AveragerTest : public ::testing::Test {
 protected:
  void SetUp() override {}
};

// Makes sure that we can compute the average of a bunch of integers.
TEST_F(AveragerTest, ComputeIntegerAverage) {
  Averager<int, 6> averager;
  for (size_t i = 0; i < averager.size(); ++i) {
    ASSERT_FALSE(averager.full());
    averager.AddData(static_cast<int>(i));
  }
  ASSERT_TRUE(averager.full());
  ASSERT_EQ(2, averager.GetAverage());
}

// Makes sure that we can compute the average of a bunch of floats.
TEST_F(AveragerTest, ComputeFloatAverage) {
  Averager<float, 100> averager;
  for (size_t i = 0; i < averager.size(); ++i) {
    ASSERT_FALSE(averager.full());
    averager.AddData(static_cast<float>(i) / 3.0);
  }
  ASSERT_TRUE(averager.full());
  ASSERT_NEAR(16.5, averager.GetAverage(), 0.001);
}

}  // namespace zeroing
}  // namespace frc971
