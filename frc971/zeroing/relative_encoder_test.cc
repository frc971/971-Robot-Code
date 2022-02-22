#include "frc971/zeroing/zeroing.h"
#include "frc971/zeroing/zeroing_test.h"
#include "gtest/gtest.h"

namespace frc971 {
namespace zeroing {
namespace testing {

class RelativeEncoderZeroingTest : public ZeroingTest {
 protected:
  void MoveTo(PositionSensorSimulator *simulator,
              RelativeEncoderZeroingEstimator *estimator, double new_position) {
    simulator->MoveTo(new_position);
    FBB fbb;
    estimator->UpdateEstimate(
        *simulator->FillSensorValues<RelativePosition>(&fbb));
  }
};

TEST_F(RelativeEncoderZeroingTest, TestRelativeEncoderZeroingWithoutMovement) {
  PositionSensorSimulator sim(1.0);
  RelativeEncoderZeroingEstimator estimator{
      constants::RelativeEncoderZeroingConstants{}};

  sim.InitializeRelativeEncoder();

  ASSERT_TRUE(estimator.zeroed());
  ASSERT_TRUE(estimator.offset_ready());
  EXPECT_DOUBLE_EQ(estimator.offset(), 0.0);
  EXPECT_DOUBLE_EQ(GetEstimatorPosition(&estimator), 0.0);

  MoveTo(&sim, &estimator, 0.1);

  EXPECT_DOUBLE_EQ(GetEstimatorPosition(&estimator), 0.1);
}

}  // namespace testing
}  // namespace zeroing
}  // namespace frc971
