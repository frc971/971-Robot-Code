#include <unistd.h>

#include <memory>

#include <random>

#include "gtest/gtest.h"
#include "frc971/zeroing/zeroing.h"
#include "frc971/control_loops/control_loops.q.h"
#include "aos/common/queue_testutils.h"
#include "aos/common/util/thread.h"
#include "aos/common/die.h"
#include "frc971/control_loops/position_sensor_sim.h"

namespace frc971 {
namespace zeroing {

using control_loops::PositionSensorSimulator;
using constants::Values;

static const size_t kSampleSize = 30;
static const double kAcceptableUnzeroedError = 0.2;

class ZeroingTest : public ::testing::Test {
 protected:
  void SetUp() override { aos::SetDieTestMode(true); }

  void MoveTo(PositionSensorSimulator* simulator, ZeroingEstimator* estimator,
              double new_position) {
    PotAndIndexPosition sensor_values_;
    simulator->MoveTo(new_position);
    simulator->GetSensorValues(&sensor_values_);
    estimator->UpdateEstimate(sensor_values_);
  }

  aos::common::testing::GlobalCoreInstance my_core;
};

TEST_F(ZeroingTest, TestMovingAverageFilter) {
  const double index_diff = 1.0;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(3.6 * index_diff, index_diff / 3.0);
  ZeroingEstimator estimator(
      Values::ZeroingConstants{kSampleSize, index_diff, 0.0});

  // The zeroing code is supposed to perform some filtering on the difference
  // between the potentiometer value and the encoder value. We assume that 300
  // samples are sufficient to have updated the filter.
  for (int i = 0; i < 300; i++) {
    MoveTo(&sim, &estimator, 3.3 * index_diff);
  }
  ASSERT_NEAR(3.3 * index_diff, estimator.position(),
              kAcceptableUnzeroedError * index_diff);

  for (int i = 0; i < 300; i++) {
    MoveTo(&sim, &estimator, 3.9 * index_diff);
  }
  ASSERT_NEAR(3.9 * index_diff, estimator.position(),
              kAcceptableUnzeroedError * index_diff);
}

TEST_F(ZeroingTest, NotZeroedBeforeEnoughSamplesCollected) {
  double index_diff = 0.5;
  double position = 3.6 * index_diff;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(position, index_diff / 3.0);
  ZeroingEstimator estimator(
      Values::ZeroingConstants{kSampleSize, index_diff, 0.0});

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
  ZeroingEstimator estimator(
      Values::ZeroingConstants{kSampleSize, index_diff, 0.0});

  // The zeroing code is supposed to perform some filtering on the difference
  // between the potentiometer value and the encoder value. We assume that 300
  // samples are sufficient to have updated the filter.
  for (int i = 0; i < 300; i++) {
    MoveTo(&sim, &estimator, 3.6);
  }
  ASSERT_NEAR(3.6, estimator.position(), kAcceptableUnzeroedError * index_diff);

  // With a single index pulse the zeroing estimator should be able to lock
  // onto the true value of the position.
  MoveTo(&sim, &estimator, 4.01);
  ASSERT_NEAR(4.01, estimator.position(), 0.001);

  MoveTo(&sim, &estimator, 4.99);
  ASSERT_NEAR(4.99, estimator.position(), 0.001);

  MoveTo(&sim, &estimator, 3.99);
  ASSERT_NEAR(3.99, estimator.position(), 0.001);

  MoveTo(&sim, &estimator, 3.01);
  ASSERT_NEAR(3.01, estimator.position(), 0.001);

  MoveTo(&sim, &estimator, 13.55);
  ASSERT_NEAR(13.55, estimator.position(), 0.001);
}

TEST_F(ZeroingTest, TestDifferentIndexDiffs) {
  double index_diff = 0.89;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(3.5 * index_diff, index_diff / 3.0);
  ZeroingEstimator estimator(
      Values::ZeroingConstants{kSampleSize, index_diff, 0.0});

  // The zeroing code is supposed to perform some filtering on the difference
  // between the potentiometer value and the encoder value. We assume that 300
  // samples are sufficient to have updated the filter.
  for (int i = 0; i < 300; i++) {
    MoveTo(&sim, &estimator, 3.5 * index_diff);
  }
  ASSERT_NEAR(3.5 * index_diff, estimator.position(),
              kAcceptableUnzeroedError * index_diff);

  // With a single index pulse the zeroing estimator should be able to lock
  // onto the true value of the position.
  MoveTo(&sim, &estimator, 4.01);
  ASSERT_NEAR(4.01, estimator.position(), 0.001);

  MoveTo(&sim, &estimator, 4.99);
  ASSERT_NEAR(4.99, estimator.position(), 0.001);

  MoveTo(&sim, &estimator, 3.99);
  ASSERT_NEAR(3.99, estimator.position(), 0.001);

  MoveTo(&sim, &estimator, 3.01);
  ASSERT_NEAR(3.01, estimator.position(), 0.001);

  MoveTo(&sim, &estimator, 13.55);
  ASSERT_NEAR(13.55, estimator.position(), 0.001);
}

TEST_F(ZeroingTest, TestPercentage) {
  double index_diff = 0.89;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(3.5 * index_diff, index_diff / 3.0);
  ZeroingEstimator estimator(
      Values::ZeroingConstants{kSampleSize, index_diff, 0.0});

  for (unsigned int i = 0; i < kSampleSize / 2; i++) {
    MoveTo(&sim, &estimator, 3.5 * index_diff);
  }
  ASSERT_NEAR(0.5, estimator.offset_ratio_ready(), 0.001);
}

TEST_F(ZeroingTest, TestOffset) {
  double index_diff = 0.89;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(3.1 * index_diff, index_diff / 3.0);
  ZeroingEstimator estimator(
      Values::ZeroingConstants{kSampleSize, index_diff, 0.0});

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
  ZeroingEstimator estimator(
      Values::ZeroingConstants{kSampleSize, index_diff, 0.0});

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
  ZeroingEstimator estimator(
      Values::ZeroingConstants{kSampleSize, index_diff, known_index_pos});

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
  ASSERT_DOUBLE_EQ(3.7 * index_diff, estimator.position());

  // Trigger one more index pulse and check the offset.
  MoveTo(&sim, &estimator, 4.7 * index_diff);
  ASSERT_DOUBLE_EQ(3.3 * index_diff, estimator.offset());
  ASSERT_DOUBLE_EQ(4.7 * index_diff, estimator.position());
}

}  // namespace zeroing
}  // namespace frc971
