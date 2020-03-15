#include "frc971/zeroing/hall_effect_and_position.h"

#include "gtest/gtest.h"

#include "frc971/zeroing/zeroing_test.h"

namespace frc971 {
namespace zeroing {
namespace testing {

class HallEffectAndPositionZeroingTest : public ZeroingTest {
 protected:
  // The starting position of the system.
  static constexpr double kStartPosition = 2.0;

  HallEffectAndPositionZeroingTest()
      : constants_(MakeConstants()), sim_(constants_.index_difference) {
    // Start the system out at the starting position.
    sim_.InitializeHallEffectAndPosition(kStartPosition,
                                         constants_.lower_hall_position,
                                         constants_.upper_hall_position);
  }

  void MoveTo(PositionSensorSimulator *simulator,
              HallEffectAndPositionZeroingEstimator *estimator,
              double new_position) {
    simulator->MoveTo(new_position);
    FBB fbb;
    estimator->UpdateEstimate(
        *simulator->FillSensorValues<HallEffectAndPosition>(&fbb));
  }

  // Returns a reasonable set of test constants.
  static constants::HallEffectZeroingConstants MakeConstants() {
    constants::HallEffectZeroingConstants constants;
    constants.lower_hall_position = 0.25;
    constants.upper_hall_position = 0.75;
    constants.index_difference = 1.0;
    constants.hall_trigger_zeroing_length = 2;
    constants.zeroing_move_direction = false;
    return constants;
  }

  // Constants, and the simulation using them.
  const constants::HallEffectZeroingConstants constants_;
  PositionSensorSimulator sim_;
};

// Tests that an error is detected when the starting position changes too much.
TEST_F(HallEffectAndPositionZeroingTest, TestHallEffectZeroing) {
  HallEffectAndPositionZeroingEstimator estimator(constants_);

  // Should not be zeroed when we stand still.
  for (int i = 0; i < 300; ++i) {
    MoveTo(&sim_, &estimator, kStartPosition);
    ASSERT_FALSE(estimator.zeroed());
  }

  MoveTo(&sim_, &estimator, 1.9);
  ASSERT_FALSE(estimator.zeroed());

  // Move to where the hall effect is triggered and make sure it becomes zeroed.
  MoveTo(&sim_, &estimator, 1.5);
  EXPECT_FALSE(estimator.zeroed());
  MoveTo(&sim_, &estimator, 1.5);
  ASSERT_TRUE(estimator.zeroed());

  // Check that the offset is calculated correctly.  We should expect to read
  // 0.5.  Since the encoder is reading -0.5 right now, the offset needs to be
  // 1.
  EXPECT_DOUBLE_EQ(1.0, estimator.offset());

  // Make sure triggering errors works.
  estimator.TriggerError();
  ASSERT_TRUE(estimator.error());

  // Ensure resetting resets the state of the estimator.
  estimator.Reset();
  ASSERT_FALSE(estimator.zeroed());
  ASSERT_FALSE(estimator.error());
}

// Tests that we don't zero on a too short pulse.
TEST_F(HallEffectAndPositionZeroingTest, TestTooShortPulse) {
  HallEffectAndPositionZeroingEstimator estimator(constants_);

  // Trigger for 1 cycle.
  MoveTo(&sim_, &estimator, 0.9);
  EXPECT_FALSE(estimator.zeroed());
  MoveTo(&sim_, &estimator, 0.5);
  EXPECT_FALSE(estimator.zeroed());
  MoveTo(&sim_, &estimator, 0.9);
  EXPECT_FALSE(estimator.zeroed());
}

// Tests that we don't zero when we go the wrong direction.
TEST_F(HallEffectAndPositionZeroingTest, TestWrongDirectionNoZero) {
  HallEffectAndPositionZeroingEstimator estimator(constants_);

  // Pass through the sensor, lingering long enough that we should zero.
  MoveTo(&sim_, &estimator, 0.0);
  ASSERT_FALSE(estimator.zeroed());
  MoveTo(&sim_, &estimator, 0.4);
  EXPECT_FALSE(estimator.zeroed());
  MoveTo(&sim_, &estimator, 0.6);
  EXPECT_FALSE(estimator.zeroed());
  MoveTo(&sim_, &estimator, 0.7);
  EXPECT_FALSE(estimator.zeroed());
  MoveTo(&sim_, &estimator, 0.9);
  EXPECT_FALSE(estimator.zeroed());
}

// Make sure we don't zero if we start in the hall effect's range.
TEST_F(HallEffectAndPositionZeroingTest, TestStartingOnNoZero) {
  HallEffectAndPositionZeroingEstimator estimator(constants_);
  MoveTo(&sim_, &estimator, 0.5);
  estimator.Reset();

  // Stay on the hall effect.  We shouldn't zero.
  EXPECT_FALSE(estimator.zeroed());
  MoveTo(&sim_, &estimator, 0.5);
  EXPECT_FALSE(estimator.zeroed());
  MoveTo(&sim_, &estimator, 0.5);
  EXPECT_FALSE(estimator.zeroed());

  // Verify moving off the hall still doesn't zero us.
  MoveTo(&sim_, &estimator, 0.0);
  EXPECT_FALSE(estimator.zeroed());
  MoveTo(&sim_, &estimator, 0.0);
  EXPECT_FALSE(estimator.zeroed());
}

}  // namespace testing
}  // namespace zeroing
}  // namespace frc971
