#include "frc971/zeroing/pulse_index.h"

#include "gtest/gtest.h"

#include "frc971/zeroing/zeroing_test.h"

namespace frc971 {
namespace zeroing {
namespace testing {

using constants::EncoderPlusIndexZeroingConstants;

class PulseIndexZeroingTest : public ZeroingTest {
 protected:
  void MoveTo(PositionSensorSimulator *simulator,
              PulseIndexZeroingEstimator *estimator, double new_position) {
    simulator->MoveTo(new_position);
    FBB fbb;
    estimator->UpdateEstimate(
        *simulator->FillSensorValues<IndexPosition>(&fbb));
  }
};

// Tests that an error is detected when the starting position changes too much.
TEST_F(PulseIndexZeroingTest, TestRelativeEncoderZeroing) {
  EncoderPlusIndexZeroingConstants constants;
  constants.index_pulse_count = 3;
  constants.index_difference = 10.0;
  constants.measured_index_position = 20.0;
  constants.known_index_pulse = 1;
  constants.allowable_encoder_error = 0.01;

  PositionSensorSimulator sim(constants.index_difference);

  const double start_pos = 2.5 * constants.index_difference;

  sim.Initialize(start_pos, constants.index_difference / 3.0,
                 constants.measured_index_position);

  PulseIndexZeroingEstimator estimator(constants);

  // Should not be zeroed when we stand still.
  for (int i = 0; i < 300; ++i) {
    MoveTo(&sim, &estimator, start_pos);
    ASSERT_FALSE(estimator.zeroed());
  }

  // Move to 1.5 constants.index_difference and we should still not be zeroed.
  MoveTo(&sim, &estimator, 1.5 * constants.index_difference);
  ASSERT_FALSE(estimator.zeroed());

  // Move to 0.5 constants.index_difference and we should still not be zeroed.
  MoveTo(&sim, &estimator, 0.5 * constants.index_difference);
  ASSERT_FALSE(estimator.zeroed());

  // Move back to 1.5 constants.index_difference and we should still not be
  // zeroed.
  MoveTo(&sim, &estimator, 1.5 * constants.index_difference);
  ASSERT_FALSE(estimator.zeroed());

  // Move back to 2.5 constants.index_difference and we should still not be
  // zeroed.
  MoveTo(&sim, &estimator, 2.5 * constants.index_difference);
  ASSERT_FALSE(estimator.zeroed());

  // Move back to 3.5 constants.index_difference and we should now be zeroed.
  MoveTo(&sim, &estimator, 3.5 * constants.index_difference);
  ASSERT_TRUE(estimator.zeroed());

  ASSERT_DOUBLE_EQ(start_pos, estimator.offset());
  ASSERT_DOUBLE_EQ(3.5 * constants.index_difference,
                   GetEstimatorPosition(&estimator));

  MoveTo(&sim, &estimator, 0.5 * constants.index_difference);
  ASSERT_DOUBLE_EQ(0.5 * constants.index_difference,
                   GetEstimatorPosition(&estimator));
}

// Tests that we can detect when an index pulse occurs where we didn't expect
// it to for the PulseIndexZeroingEstimator.
TEST_F(PulseIndexZeroingTest, TestRelativeEncoderSlipping) {
  EncoderPlusIndexZeroingConstants constants;
  constants.index_pulse_count = 3;
  constants.index_difference = 10.0;
  constants.measured_index_position = 20.0;
  constants.known_index_pulse = 1;
  constants.allowable_encoder_error = 0.05;

  PositionSensorSimulator sim(constants.index_difference);

  const double start_pos =
      constants.measured_index_position + 0.5 * constants.index_difference;

  for (double direction : {1.0, -1.0}) {
    sim.Initialize(start_pos, constants.index_difference / 3.0,
                   constants.measured_index_position);

    PulseIndexZeroingEstimator estimator(constants);

    // Zero the estimator.
    MoveTo(&sim, &estimator, start_pos - 1 * constants.index_difference);
    MoveTo(
        &sim, &estimator,
        start_pos - constants.index_pulse_count * constants.index_difference);
    ASSERT_TRUE(estimator.zeroed());
    ASSERT_FALSE(estimator.error());

    // We have a 5% allowable error so we slip a little bit each time and make
    // sure that the index pulses are still accepted.
    for (double error = 0.00;
         ::std::abs(error) < constants.allowable_encoder_error;
         error += 0.01 * direction) {
      sim.Initialize(start_pos, constants.index_difference / 3.0,
                     constants.measured_index_position +
                         error * constants.index_difference);
      MoveTo(&sim, &estimator, start_pos - constants.index_difference);
      EXPECT_FALSE(estimator.error());
    }

    // As soon as we hit cross the error margin, we should trigger an error.
    sim.Initialize(start_pos, constants.index_difference / 3.0,
                   constants.measured_index_position +
                       constants.allowable_encoder_error * 1.1 *
                           constants.index_difference * direction);
    MoveTo(&sim, &estimator, start_pos - constants.index_difference);
    ASSERT_TRUE(estimator.error());
  }
}

}  // namespace testing
}  // namespace zeroing
}  // namespace frc971
