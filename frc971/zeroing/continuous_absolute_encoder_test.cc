#include "frc971/zeroing/continuous_absolute_encoder.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "frc971/zeroing/wrap.h"
#include "frc971/zeroing/zeroing_test.h"

namespace frc971 {
namespace zeroing {
namespace testing {

using constants::ContinuousAbsoluteEncoderZeroingConstants;

class ContinuousAbsoluteEncoderZeroingTest : public ZeroingTest {
 protected:
  void MoveTo(PositionSensorSimulator *simulator,
              ContinuousAbsoluteEncoderZeroingEstimator *estimator,
              double new_position) {
    simulator->MoveTo(new_position);
    flatbuffers::FlatBufferBuilder fbb;
    estimator->UpdateEstimate(
        *simulator->FillSensorValues<AbsolutePosition>(&fbb));
  }
};

// Makes sure that using an absolute encoder lets us zero without moving.
TEST_F(ContinuousAbsoluteEncoderZeroingTest,
       TestContinuousAbsoluteEncoderZeroingWithoutMovement) {
  const double index_diff = 1.0;
  PositionSensorSimulator sim(index_diff);

  const double start_pos = 2.1;
  double measured_absolute_position = 0.3 * index_diff;

  ContinuousAbsoluteEncoderZeroingConstants constants{
      kSampleSize, index_diff,        measured_absolute_position,
      0.1,         kMovingBufferSize, kIndexErrorFraction};

  sim.Initialize(start_pos, index_diff / 3.0, 0.0,
                 constants.measured_absolute_position);

  ContinuousAbsoluteEncoderZeroingEstimator estimator(constants);

  for (size_t i = 0; i < kSampleSize + kMovingBufferSize - 1; ++i) {
    MoveTo(&sim, &estimator, start_pos);
    ASSERT_FALSE(estimator.zeroed());
  }

  MoveTo(&sim, &estimator, start_pos);
  ASSERT_TRUE(estimator.zeroed());
  // Because the continuous estimator doesn't care about extra revolutions, it
  // will have brought the offset down to less than index_diff.
  EXPECT_NEAR(Wrap(0.0, start_pos, index_diff), estimator.offset(), 1e-12);
}

// Makes sure that if the underlying mechanism moves by >1 revolution that the
// continuous zeroing estimator handles it correctly.
TEST_F(ContinuousAbsoluteEncoderZeroingTest,
       ContinuousEstimatorZeroesAcrossRevolution) {
  const double index_diff = 1.0;
  PositionSensorSimulator sim(index_diff);

  const double start_pos = 2.1;
  double measured_absolute_position = 0.3 * index_diff;

  ContinuousAbsoluteEncoderZeroingConstants constants{
      kSampleSize, index_diff,        measured_absolute_position,
      0.1,         kMovingBufferSize, kIndexErrorFraction};

  sim.Initialize(start_pos, index_diff / 3.0, 0.0,
                 constants.measured_absolute_position);

  ContinuousAbsoluteEncoderZeroingEstimator estimator(constants);

  for (size_t i = 0; i < kSampleSize + kMovingBufferSize - 1; ++i) {
    MoveTo(&sim, &estimator, start_pos);
    ASSERT_FALSE(estimator.zeroed());
  }

  MoveTo(&sim, &estimator, start_pos);
  ASSERT_TRUE(estimator.zeroed());
  // Because the continuous estimator doesn't care about extra revolutions, it
  // will have brought the offset down to less than index_diff.
  EXPECT_NEAR(Wrap(0.0, start_pos, index_diff), estimator.offset(), 1e-12);

  // Now, rotate by a full revolution, then stay still. We should stay zeroed.
  for (size_t i = 0; i < kSampleSize + kMovingBufferSize; ++i) {
    MoveTo(&sim, &estimator, start_pos + 10 * index_diff);
  }
  ASSERT_TRUE(estimator.zeroed());
  ASSERT_FALSE(estimator.error());
}

// Makes sure that we ignore a NAN if we get it, but will correctly zero
// afterwards.
TEST_F(ContinuousAbsoluteEncoderZeroingTest,
       TestContinuousAbsoluteEncoderZeroingIgnoresNAN) {
  const double index_diff = 1.0;
  PositionSensorSimulator sim(index_diff);

  const double start_pos = 2.1;
  double measured_absolute_position = 0.3 * index_diff;

  ContinuousAbsoluteEncoderZeroingConstants constants{
      kSampleSize, index_diff,        measured_absolute_position,
      0.1,         kMovingBufferSize, kIndexErrorFraction};

  sim.Initialize(start_pos, index_diff / 3.0, 0.0,
                 constants.measured_absolute_position);

  ContinuousAbsoluteEncoderZeroingEstimator estimator(constants);

  // We tolerate a couple NANs before we start.
  flatbuffers::FlatBufferBuilder fbb;
  fbb.Finish(CreateAbsolutePosition(
      fbb, 0.0, ::std::numeric_limits<double>::quiet_NaN()));
  const auto sensor_values =
      flatbuffers::GetRoot<AbsolutePosition>(fbb.GetBufferPointer());
  for (size_t i = 0; i < kSampleSize - 1; ++i) {
    estimator.UpdateEstimate(*sensor_values);
  }

  for (size_t i = 0; i < kSampleSize + kMovingBufferSize - 1; ++i) {
    MoveTo(&sim, &estimator, start_pos);
    ASSERT_FALSE(estimator.zeroed());
  }

  MoveTo(&sim, &estimator, start_pos);
  ASSERT_TRUE(estimator.zeroed());
  // Because the continuous estimator doesn't care about extra revolutions, it
  // will have brought the offset down to less than index_diff.
  EXPECT_NEAR(Wrap(0.0, start_pos, index_diff), estimator.offset(), 1e-12);
}

// Makes sure that using an absolute encoder doesn't let us zero while moving.
TEST_F(ContinuousAbsoluteEncoderZeroingTest,
       TestContinuousAbsoluteEncoderZeroingWithMovement) {
  const double index_diff = 1.0;
  PositionSensorSimulator sim(index_diff);

  const double start_pos = 10 * index_diff;
  double measured_absolute_position = 0.3 * index_diff;

  ContinuousAbsoluteEncoderZeroingConstants constants{
      kSampleSize, index_diff,        measured_absolute_position,
      0.1,         kMovingBufferSize, kIndexErrorFraction};

  sim.Initialize(start_pos, index_diff / 3.0, 0.0,
                 constants.measured_absolute_position);

  ContinuousAbsoluteEncoderZeroingEstimator estimator(constants);

  for (size_t i = 0; i < kSampleSize + kMovingBufferSize - 1; ++i) {
    MoveTo(&sim, &estimator, start_pos + i * index_diff);
    ASSERT_FALSE(estimator.zeroed());
  }
  MoveTo(&sim, &estimator, start_pos + 10 * index_diff);

  MoveTo(&sim, &estimator, start_pos);
  ASSERT_FALSE(estimator.zeroed());
}

// Makes sure we detect an error if the ZeroingEstimator gets sent a NaN.
TEST_F(ContinuousAbsoluteEncoderZeroingTest,
       TestContinuousAbsoluteEncoderZeroingWithNaN) {
  ContinuousAbsoluteEncoderZeroingConstants constants{
      kSampleSize, 1, 0.3, 0.1, kMovingBufferSize, kIndexErrorFraction};

  ContinuousAbsoluteEncoderZeroingEstimator estimator(constants);

  flatbuffers::FlatBufferBuilder fbb;
  fbb.Finish(CreateAbsolutePosition(
      fbb, 0.0, ::std::numeric_limits<double>::quiet_NaN()));
  const auto sensor_values =
      flatbuffers::GetRoot<AbsolutePosition>(fbb.GetBufferPointer());
  for (size_t i = 0; i < kSampleSize - 1; ++i) {
    estimator.UpdateEstimate(*sensor_values);
  }
  ASSERT_FALSE(estimator.error());

  estimator.UpdateEstimate(*sensor_values);
  ASSERT_TRUE(estimator.error());

  flatbuffers::FlatBufferBuilder fbb2;
  fbb2.Finish(estimator.GetEstimatorState(&fbb2));

  const AbsoluteEncoderEstimatorState *state =
      flatbuffers::GetRoot<AbsoluteEncoderEstimatorState>(
          fbb2.GetBufferPointer());

  EXPECT_THAT(*state->errors(),
              ::testing::ElementsAre(ZeroingError::LOST_ABSOLUTE_ENCODER));
}

}  // namespace testing
}  // namespace zeroing
}  // namespace frc971
