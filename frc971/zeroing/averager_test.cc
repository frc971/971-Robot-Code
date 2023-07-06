#include "frc971/zeroing/averager.h"

#include "gtest/gtest.h"

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
  ASSERT_EQ(2, averager.GetAverage()(0, 0));
}

// Makes sure that we can compute the average of a bunch of floats.
TEST_F(AveragerTest, ComputeFloatAverage) {
  Averager<float, 100> averager;
  for (size_t i = 0; i < averager.size(); ++i) {
    ASSERT_FALSE(averager.full());
    averager.AddData(static_cast<float>(i) / 3.0);
  }
  ASSERT_TRUE(averager.full());
  ASSERT_NEAR(16.5, averager.GetAverage()(0, 0), 0.001);
}

TEST_F(AveragerTest, CalculateRange) {
  Averager<float, 5, 2> averager;
  ASSERT_EQ(0, averager.GetRange());
  averager.AddData({100, 10});
  averager.AddData({105, 15});
  averager.AddData({90, 9});
  ASSERT_EQ(15, averager.GetRange());
  for (size_t ii = 0; ii < averager.size(); ++ii) {
    averager.AddData({10, 20});
  }
  ASSERT_EQ(0, averager.GetRange());
}

TEST_F(AveragerTest, ResetAverager) {
  Averager<float, 5> averager;
  for (size_t ii = 0; ii < averager.size(); ++ii) {
    averager.AddData(10);
  }
  ASSERT_TRUE(averager.full());
  ASSERT_EQ(10.0, averager.GetAverage()(0, 0));
  averager.Reset();
  ASSERT_FALSE(averager.full());
  ASSERT_EQ(0.0, averager.GetAverage()(0, 0));
}

}  // namespace zeroing
}  // namespace frc971
