#include <unistd.h>

#include <memory>

#include <random>

#include "gtest/gtest.h"
#include "frc971/zeroing/zeroing.h"
#include "frc971/control_loops/control_loops.q.h"
#include "aos/testing/test_shm.h"
#include "aos/common/util/thread.h"
#include "aos/common/die.h"
#include "frc971/control_loops/position_sensor_sim.h"

namespace frc971 {
namespace zeroing {

using control_loops::PositionSensorSimulator;
using constants::PotAndIndexPulseZeroingConstants;
using constants::PotAndAbsoluteEncoderZeroingConstants;
using constants::EncoderPlusIndexZeroingConstants;

static const size_t kSampleSize = 30;
static const double kAcceptableUnzeroedError = 0.2;
static const double kIndexErrorFraction = 0.3;
static const size_t kMovingBufferSize = 3;

class ZeroingTest : public ::testing::Test {
 protected:
  void SetUp() override { aos::SetDieTestMode(true); }

  void MoveTo(PositionSensorSimulator *simulator,
              PotAndIndexPulseZeroingEstimator *estimator,
              double new_position) {
    PotAndIndexPosition sensor_values;
    simulator->MoveTo(new_position);
    simulator->GetSensorValues(&sensor_values);
    estimator->UpdateEstimate(sensor_values);
  }

  void MoveTo(PositionSensorSimulator *simulator,
              PotAndAbsEncoderZeroingEstimator *estimator,
              double new_position) {
    PotAndAbsolutePosition sensor_values_;
    simulator->MoveTo(new_position);
    simulator->GetSensorValues(&sensor_values_);
    estimator->UpdateEstimate(sensor_values_);
  }

  void MoveTo(PositionSensorSimulator *simulator,
              PulseIndexZeroingEstimator *estimator, double new_position) {
    IndexPosition sensor_values_;
    simulator->MoveTo(new_position);
    simulator->GetSensorValues(&sensor_values_);
    estimator->UpdateEstimate(sensor_values_);
  }

  void MoveTo(PositionSensorSimulator *simulator,
              HallEffectAndPositionZeroingEstimator *estimator,
              double new_position) {
    HallEffectAndPosition sensor_values_;
    simulator->MoveTo(new_position);
    simulator->GetSensorValues(&sensor_values_);
    estimator->UpdateEstimate(sensor_values_);
  }

  ::aos::testing::TestSharedMemory my_shm_;
};

TEST_F(ZeroingTest, TestMovingAverageFilter) {
  const double index_diff = 1.0;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(3.6 * index_diff, index_diff / 3.0);
  PotAndIndexPulseZeroingEstimator estimator(PotAndIndexPulseZeroingConstants{
      kSampleSize, index_diff, 0.0, kIndexErrorFraction});

  // The zeroing code is supposed to perform some filtering on the difference
  // between the potentiometer value and the encoder value. We assume that 300
  // samples are sufficient to have updated the filter.
  for (int i = 0; i < 300; i++) {
    MoveTo(&sim, &estimator, 3.3 * index_diff);
  }
  ASSERT_NEAR(3.3 * index_diff, estimator.GetEstimatorState().position,
              kAcceptableUnzeroedError * index_diff);

  for (int i = 0; i < 300; i++) {
    MoveTo(&sim, &estimator, 3.9 * index_diff);
  }
  ASSERT_NEAR(3.9 * index_diff, estimator.GetEstimatorState().position,
              kAcceptableUnzeroedError * index_diff);
}

TEST_F(ZeroingTest, NotZeroedBeforeEnoughSamplesCollected) {
  double index_diff = 0.5;
  double position = 3.6 * index_diff;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(position, index_diff / 3.0);
  PotAndIndexPulseZeroingEstimator estimator(PotAndIndexPulseZeroingConstants{
      kSampleSize, index_diff, 0.0, kIndexErrorFraction});

  // Make sure that the zeroing code does not consider itself zeroed until we
  // collect a good amount of samples. In this case we're waiting until the
  // moving average filter is full.
  for (unsigned int i = 0; i < kSampleSize - 1; i++) {
    MoveTo(&sim, &estimator, position += index_diff);
    ASSERT_FALSE(estimator.zeroed());
  }

  MoveTo(&sim, &estimator, position);
  ASSERT_TRUE(estimator.zeroed());
}

TEST_F(ZeroingTest, TestLotsOfMovement) {
  double index_diff = 1.0;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(3.6, index_diff / 3.0);
  PotAndIndexPulseZeroingEstimator estimator(PotAndIndexPulseZeroingConstants{
      kSampleSize, index_diff, 0.0, kIndexErrorFraction});

  // The zeroing code is supposed to perform some filtering on the difference
  // between the potentiometer value and the encoder value. We assume that 300
  // samples are sufficient to have updated the filter.
  for (int i = 0; i < 300; i++) {
    MoveTo(&sim, &estimator, 3.6);
  }
  ASSERT_NEAR(3.6, estimator.GetEstimatorState().position,
              kAcceptableUnzeroedError * index_diff);

  // With a single index pulse the zeroing estimator should be able to lock
  // onto the true value of the position.
  MoveTo(&sim, &estimator, 4.01);
  ASSERT_NEAR(4.01, estimator.GetEstimatorState().position, 0.001);

  MoveTo(&sim, &estimator, 4.99);
  ASSERT_NEAR(4.99, estimator.GetEstimatorState().position, 0.001);

  MoveTo(&sim, &estimator, 3.99);
  ASSERT_NEAR(3.99, estimator.GetEstimatorState().position, 0.001);

  MoveTo(&sim, &estimator, 3.01);
  ASSERT_NEAR(3.01, estimator.GetEstimatorState().position, 0.001);

  MoveTo(&sim, &estimator, 13.55);
  ASSERT_NEAR(13.55, estimator.GetEstimatorState().position, 0.001);
}

TEST_F(ZeroingTest, TestDifferentIndexDiffs) {
  double index_diff = 0.89;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(3.5 * index_diff, index_diff / 3.0);
  PotAndIndexPulseZeroingEstimator estimator(PotAndIndexPulseZeroingConstants{
      kSampleSize, index_diff, 0.0, kIndexErrorFraction});

  // The zeroing code is supposed to perform some filtering on the difference
  // between the potentiometer value and the encoder value. We assume that 300
  // samples are sufficient to have updated the filter.
  for (int i = 0; i < 300; i++) {
    MoveTo(&sim, &estimator, 3.5 * index_diff);
  }
  ASSERT_NEAR(3.5 * index_diff, estimator.GetEstimatorState().position,
              kAcceptableUnzeroedError * index_diff);

  // With a single index pulse the zeroing estimator should be able to lock
  // onto the true value of the position.
  MoveTo(&sim, &estimator, 4.01);
  ASSERT_NEAR(4.01, estimator.GetEstimatorState().position, 0.001);

  MoveTo(&sim, &estimator, 4.99);
  ASSERT_NEAR(4.99, estimator.GetEstimatorState().position, 0.001);

  MoveTo(&sim, &estimator, 3.99);
  ASSERT_NEAR(3.99, estimator.GetEstimatorState().position, 0.001);

  MoveTo(&sim, &estimator, 3.01);
  ASSERT_NEAR(3.01, estimator.GetEstimatorState().position, 0.001);

  MoveTo(&sim, &estimator, 13.55);
  ASSERT_NEAR(13.55, estimator.GetEstimatorState().position, 0.001);
}

TEST_F(ZeroingTest, TestPercentage) {
  double index_diff = 0.89;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(3.5 * index_diff, index_diff / 3.0);
  PotAndIndexPulseZeroingEstimator estimator(PotAndIndexPulseZeroingConstants{
      kSampleSize, index_diff, 0.0, kIndexErrorFraction});

  for (unsigned int i = 0; i < kSampleSize / 2; i++) {
    MoveTo(&sim, &estimator, 3.5 * index_diff);
  }
  ASSERT_NEAR(0.5, estimator.offset_ratio_ready(), 0.001);
  ASSERT_FALSE(estimator.offset_ready());

  for (unsigned int i = 0; i < kSampleSize / 2; i++) {
    MoveTo(&sim, &estimator, 3.5 * index_diff);
  }
  ASSERT_NEAR(1.0, estimator.offset_ratio_ready(), 0.001);
  ASSERT_TRUE(estimator.offset_ready());
}

TEST_F(ZeroingTest, TestOffset) {
  double index_diff = 0.89;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(3.1 * index_diff, index_diff / 3.0);
  PotAndIndexPulseZeroingEstimator estimator(PotAndIndexPulseZeroingConstants{
      kSampleSize, index_diff, 0.0, kIndexErrorFraction});

  MoveTo(&sim, &estimator, 3.1 * index_diff);

  for (unsigned int i = 0; i < kSampleSize; i++) {
    MoveTo(&sim, &estimator, 5.0 * index_diff);
  }

  ASSERT_NEAR(3.1 * index_diff, estimator.offset(), 0.001);
}

TEST_F(ZeroingTest, WaitForIndexPulseAfterReset) {
  double index_diff = 0.6;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(3.1 * index_diff, index_diff / 3.0);
  PotAndIndexPulseZeroingEstimator estimator(PotAndIndexPulseZeroingConstants{
      kSampleSize, index_diff, 0.0, kIndexErrorFraction});

  // Make sure to fill up the averaging filter with samples.
  for (unsigned int i = 0; i < kSampleSize; i++) {
    MoveTo(&sim, &estimator, 3.1 * index_diff);
  }

  // Make sure we're not zeroed until we hit an index pulse.
  ASSERT_FALSE(estimator.zeroed());

  // Trigger an index pulse; we should now be zeroed.
  MoveTo(&sim, &estimator, 4.5 * index_diff);
  ASSERT_TRUE(estimator.zeroed());

  // Reset the zeroing logic and supply a bunch of samples within the current
  // index segment.
  estimator.Reset();
  for (unsigned int i = 0; i < kSampleSize; i++) {
    MoveTo(&sim, &estimator, 4.2 * index_diff);
  }

  // Make sure we're not zeroed until we hit an index pulse.
  ASSERT_FALSE(estimator.zeroed());

  // Trigger another index pulse; we should be zeroed again.
  MoveTo(&sim, &estimator, 3.1 * index_diff);
  ASSERT_TRUE(estimator.zeroed());
}

TEST_F(ZeroingTest, TestNonZeroIndexPulseOffsets) {
  const double index_diff = 0.9;
  const double known_index_pos = 3.5 * index_diff;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(3.3 * index_diff, index_diff / 3.0, known_index_pos);
  PotAndIndexPulseZeroingEstimator estimator(PotAndIndexPulseZeroingConstants{
      kSampleSize, index_diff, known_index_pos, kIndexErrorFraction});

  // Make sure to fill up the averaging filter with samples.
  for (unsigned int i = 0; i < kSampleSize; i++) {
    MoveTo(&sim, &estimator, 3.3 * index_diff);
  }

  // Make sure we're not zeroed until we hit an index pulse.
  ASSERT_FALSE(estimator.zeroed());

  // Trigger an index pulse; we should now be zeroed.
  MoveTo(&sim, &estimator, 3.7 * index_diff);
  ASSERT_TRUE(estimator.zeroed());
  ASSERT_DOUBLE_EQ(3.3 * index_diff, estimator.offset());
  ASSERT_DOUBLE_EQ(3.7 * index_diff, estimator.GetEstimatorState().position);

  // Trigger one more index pulse and check the offset.
  MoveTo(&sim, &estimator, 4.7 * index_diff);
  ASSERT_DOUBLE_EQ(3.3 * index_diff, estimator.offset());
  ASSERT_DOUBLE_EQ(4.7 * index_diff, estimator.GetEstimatorState().position);
}

TEST_F(ZeroingTest, BasicErrorAPITest) {
  const double index_diff = 1.0;
  PotAndIndexPulseZeroingEstimator estimator(PotAndIndexPulseZeroingConstants{
      kSampleSize, index_diff, 0.0, kIndexErrorFraction});
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(1.5 * index_diff, index_diff / 3.0, 0.0);

  // Perform a simple move and make sure that no error occured.
  MoveTo(&sim, &estimator, 3.5 * index_diff);
  ASSERT_FALSE(estimator.error());

  // Trigger an error and make sure it's reported.
  estimator.TriggerError();
  ASSERT_TRUE(estimator.error());

  // Make sure that it can recover after a reset.
  estimator.Reset();
  ASSERT_FALSE(estimator.error());
  MoveTo(&sim, &estimator, 4.5 * index_diff);
  MoveTo(&sim, &estimator, 5.5 * index_diff);
  ASSERT_FALSE(estimator.error());
}

// Tests that an error is detected when the starting position changes too much.
TEST_F(ZeroingTest, TestIndexOffsetError) {
  const double index_diff = 0.8;
  const double known_index_pos = 2 * index_diff;
  const size_t sample_size = 30;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(10 * index_diff, index_diff / 3.0, known_index_pos);
  PotAndIndexPulseZeroingEstimator estimator(PotAndIndexPulseZeroingConstants{
      sample_size, index_diff, known_index_pos, kIndexErrorFraction});

  for (size_t i = 0; i < sample_size; i++) {
    MoveTo(&sim, &estimator, 13 * index_diff);
  }
  MoveTo(&sim, &estimator, 8 * index_diff);

  ASSERT_TRUE(estimator.zeroed());
  ASSERT_FALSE(estimator.error());
  sim.Initialize(9.0 * index_diff + 0.31 * index_diff, index_diff / 3.0,
                 known_index_pos);
  MoveTo(&sim, &estimator, 9 * index_diff);
  ASSERT_TRUE(estimator.zeroed());
  ASSERT_TRUE(estimator.error());
}

// Makes sure that using an absolute encoder lets us zero without moving.
TEST_F(ZeroingTest, TestAbsoluteEncoderZeroingWithoutMovement) {
  const double index_diff = 1.0;
  PositionSensorSimulator sim(index_diff);

  const double start_pos = 2.1;
  double measured_absolute_position = 0.3 * index_diff;

  PotAndAbsoluteEncoderZeroingConstants constants{
      kSampleSize, index_diff,        measured_absolute_position,
      0.1,         kMovingBufferSize, kIndexErrorFraction};

  sim.Initialize(start_pos, index_diff / 3.0, 0.0,
                 constants.measured_absolute_position);

  PotAndAbsEncoderZeroingEstimator estimator(constants);

  for (size_t i = 0; i < kSampleSize + kMovingBufferSize - 1; ++i) {
    MoveTo(&sim, &estimator, start_pos);
    ASSERT_FALSE(estimator.zeroed());
  }

  MoveTo(&sim, &estimator, start_pos);
  ASSERT_TRUE(estimator.zeroed());
  EXPECT_DOUBLE_EQ(start_pos, estimator.offset());
}

// Makes sure that using an absolute encoder doesn't let us zero while moving.
TEST_F(ZeroingTest, TestAbsoluteEncoderZeroingWithMovement) {
  const double index_diff = 1.0;
  PositionSensorSimulator sim(index_diff);

  const double start_pos = 10 * index_diff;
  double measured_absolute_position = 0.3 * index_diff;

  PotAndAbsoluteEncoderZeroingConstants constants{
      kSampleSize, index_diff,        measured_absolute_position,
      0.1,         kMovingBufferSize, kIndexErrorFraction};

  sim.Initialize(start_pos, index_diff / 3.0, 0.0,
                 constants.measured_absolute_position);

  PotAndAbsEncoderZeroingEstimator estimator(constants);

  for (size_t i = 0; i < kSampleSize + kMovingBufferSize - 1; ++i) {
    MoveTo(&sim, &estimator, start_pos + i * index_diff);
    ASSERT_FALSE(estimator.zeroed());
  }
  MoveTo(&sim, &estimator, start_pos + 10 * index_diff);

  MoveTo(&sim, &estimator, start_pos);
  ASSERT_FALSE(estimator.zeroed());
}

// Makes sure we detect an error if the ZeroingEstimator gets sent a NaN.
TEST_F(ZeroingTest, TestAbsoluteEncoderZeroingWithNaN) {
  PotAndAbsoluteEncoderZeroingConstants constants{
      kSampleSize, 1, 0.3, 0.1, kMovingBufferSize, kIndexErrorFraction};

  PotAndAbsEncoderZeroingEstimator estimator(constants);

  PotAndAbsolutePosition sensor_values_;
  sensor_values_.absolute_encoder = ::std::numeric_limits<double>::quiet_NaN();
  sensor_values_.encoder = 0.0;
  sensor_values_.pot = 0.0;
  estimator.UpdateEstimate(sensor_values_);

  ASSERT_TRUE(estimator.error());
}

// Tests that an error is detected when the starting position changes too much.
TEST_F(ZeroingTest, TestRelativeEncoderZeroing) {
  EncoderPlusIndexZeroingConstants constants;
  constants.index_pulse_count = 3;
  constants.index_difference = 10.0;
  constants.measured_index_position = 20.0;
  constants.known_index_pulse = 1;

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
                   estimator.GetEstimatorState().position);

  MoveTo(&sim, &estimator, 0.5 * constants.index_difference);
  ASSERT_DOUBLE_EQ(0.5 * constants.index_difference,
                   estimator.GetEstimatorState().position);
}

// Tests that an error is detected when the starting position changes too much.
TEST_F(ZeroingTest, TestHallEffectZeroing) {
  constants::HallEffectZeroingConstants constants;
  constants.lower_hall_position = 0.25;
  constants.upper_hall_position = 0.75;
  constants.index_difference = 1.0;
  constants.hall_trigger_zeroing_length = 2;
  constants.zeroing_move_direction = false;

  PositionSensorSimulator sim(constants.index_difference);

  const double start_pos = 1.0;

  sim.InitializeHallEffectAndPosition(start_pos, constants.lower_hall_position,
                                      constants.upper_hall_position);

  HallEffectAndPositionZeroingEstimator estimator(constants);

  // Should not be zeroed when we stand still.
  for (int i = 0; i < 300; ++i) {
    MoveTo(&sim, &estimator, start_pos);
    ASSERT_FALSE(estimator.zeroed());
  }

  MoveTo(&sim, &estimator, 0.9);
  ASSERT_FALSE(estimator.zeroed());

  // Move to where the hall effect is triggered and make sure it becomes zeroed.
  MoveTo(&sim, &estimator, 0.5);
  EXPECT_FALSE(estimator.zeroed());
  MoveTo(&sim, &estimator, 0.5);
  ASSERT_TRUE(estimator.zeroed());

  // Check that the offset is calculated correctly.
  EXPECT_DOUBLE_EQ(-0.25, estimator.offset());

  // Make sure triggering errors works.
  estimator.TriggerError();
  ASSERT_TRUE(estimator.error());

  // Ensure resetting resets the state of the estimator.
  estimator.Reset();
  ASSERT_FALSE(estimator.zeroed());
  ASSERT_FALSE(estimator.error());

  // Make sure we don't become zeroed if the hall effect doesn't trigger for
  // long enough.
  MoveTo(&sim, &estimator, 0.9);
  EXPECT_FALSE(estimator.zeroed());
  MoveTo(&sim, &estimator, 0.5);
  EXPECT_FALSE(estimator.zeroed());
  MoveTo(&sim, &estimator, 0.9);
  EXPECT_FALSE(estimator.zeroed());

  // Make sure we can zero moving in the opposite direction as before and stay
  // zeroed once the hall effect is no longer triggered.

  MoveTo(&sim, &estimator, 0.0);
  ASSERT_FALSE(estimator.zeroed());
  MoveTo(&sim, &estimator, 0.4);
  EXPECT_FALSE(estimator.zeroed());
  MoveTo(&sim, &estimator, 0.6);
  EXPECT_FALSE(estimator.zeroed());
  MoveTo(&sim, &estimator, 0.9);
  EXPECT_FALSE(estimator.zeroed());

  // Check that the offset is calculated correctly.
  EXPECT_DOUBLE_EQ(-0.75, estimator.offset());

  // Make sure we don't zero if we start in the hall effect's range, before we
  // reset, we also check that there were no errors.
  MoveTo(&sim, &estimator, 0.5);
  ASSERT_TRUE(estimator.zeroed());
  ASSERT_FALSE(estimator.error());
  estimator.Reset();
  EXPECT_FALSE(estimator.zeroed());
  MoveTo(&sim, &estimator, 0.5);
  EXPECT_FALSE(estimator.zeroed());
  MoveTo(&sim, &estimator, 0.5);
  EXPECT_FALSE(estimator.zeroed());
}


}  // namespace zeroing
}  // namespace frc971
