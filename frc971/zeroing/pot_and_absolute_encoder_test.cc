#include "frc971/zeroing/pot_and_absolute_encoder.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "frc971/zeroing/zeroing_test.h"

namespace frc971 {
namespace zeroing {
namespace testing {

using constants::PotAndAbsoluteEncoderZeroingConstants;

class PotAndAbsoluteEncoderZeroingTest : public ZeroingTest {
 protected:
  void MoveTo(PositionSensorSimulator *simulator,
              PotAndAbsoluteEncoderZeroingEstimator *estimator,
              double new_position) {
    simulator->MoveTo(new_position);
    flatbuffers::FlatBufferBuilder fbb;
    estimator->UpdateEstimate(
        *simulator->FillSensorValues<PotAndAbsolutePosition>(&fbb));
  }
};

// Makes sure that using an absolute encoder lets us zero without moving.
TEST_F(PotAndAbsoluteEncoderZeroingTest,
       TestPotAndAbsoluteEncoderZeroingWithoutMovement) {
  const double index_diff = 1.0;
  PositionSensorSimulator sim(index_diff);

  const double start_pos = 2.1;
  double measured_absolute_position = 0.3 * index_diff;

  PotAndAbsoluteEncoderZeroingConstants constants{
      kSampleSize, index_diff,        measured_absolute_position,
      0.1,         kMovingBufferSize, kIndexErrorFraction};

  sim.Initialize(start_pos, index_diff / 3.0, 0.0,
                 constants.measured_absolute_position);

  PotAndAbsoluteEncoderZeroingEstimator estimator(constants);

  for (size_t i = 0; i < kSampleSize + kMovingBufferSize - 1; ++i) {
    MoveTo(&sim, &estimator, start_pos);
    ASSERT_FALSE(estimator.zeroed());
  }

  MoveTo(&sim, &estimator, start_pos);
  ASSERT_TRUE(estimator.zeroed());
  EXPECT_DOUBLE_EQ(start_pos, estimator.offset());
}

// Makes sure that we ignore a NAN if we get it, but will correctly zero
// afterwards.
TEST_F(PotAndAbsoluteEncoderZeroingTest,
       TestPotAndAbsoluteEncoderZeroingIgnoresNAN) {
  const double index_diff = 1.0;
  PositionSensorSimulator sim(index_diff);

  const double start_pos = 2.1;
  double measured_absolute_position = 0.3 * index_diff;

  PotAndAbsoluteEncoderZeroingConstants constants{
      kSampleSize, index_diff,        measured_absolute_position,
      0.1,         kMovingBufferSize, kIndexErrorFraction};

  sim.Initialize(start_pos, index_diff / 3.0, 0.0,
                 constants.measured_absolute_position);

  PotAndAbsoluteEncoderZeroingEstimator estimator(constants);

  // We tolerate a couple NANs before we start.
  flatbuffers::FlatBufferBuilder fbb;
  fbb.Finish(CreatePotAndAbsolutePosition(
      fbb, 0.0, ::std::numeric_limits<double>::quiet_NaN(), 0.0));
  for (size_t i = 0; i < kSampleSize - 1; ++i) {
    estimator.UpdateEstimate(
        *flatbuffers::GetRoot<PotAndAbsolutePosition>(fbb.GetBufferPointer()));
  }

  for (size_t i = 0; i < kSampleSize + kMovingBufferSize - 1; ++i) {
    MoveTo(&sim, &estimator, start_pos);
    ASSERT_FALSE(estimator.zeroed());
  }

  MoveTo(&sim, &estimator, start_pos);
  ASSERT_TRUE(estimator.zeroed());
  EXPECT_DOUBLE_EQ(start_pos, estimator.offset());
}

// Makes sure that using an absolute encoder doesn't let us zero while moving.
TEST_F(PotAndAbsoluteEncoderZeroingTest,
       TestPotAndAbsoluteEncoderZeroingWithMovement) {
  const double index_diff = 1.0;
  PositionSensorSimulator sim(index_diff);

  const double start_pos = 10 * index_diff;
  double measured_absolute_position = 0.3 * index_diff;

  PotAndAbsoluteEncoderZeroingConstants constants{
      kSampleSize, index_diff,        measured_absolute_position,
      0.1,         kMovingBufferSize, kIndexErrorFraction};

  sim.Initialize(start_pos, index_diff / 3.0, 0.0,
                 constants.measured_absolute_position);

  PotAndAbsoluteEncoderZeroingEstimator estimator(constants);

  for (size_t i = 0; i < kSampleSize + kMovingBufferSize - 1; ++i) {
    MoveTo(&sim, &estimator, start_pos + i * index_diff);
    ASSERT_FALSE(estimator.zeroed());
  }
  MoveTo(&sim, &estimator, start_pos + 10 * index_diff);

  MoveTo(&sim, &estimator, start_pos);
  ASSERT_FALSE(estimator.zeroed());
}

// Makes sure we detect an error if the ZeroingEstimator gets sent a NaN.
TEST_F(PotAndAbsoluteEncoderZeroingTest,
       TestPotAndAbsoluteEncoderZeroingWithNaN) {
  PotAndAbsoluteEncoderZeroingConstants constants{
      kSampleSize, 1, 0.3, 0.1, kMovingBufferSize, kIndexErrorFraction};

  PotAndAbsoluteEncoderZeroingEstimator estimator(constants);

  flatbuffers::FlatBufferBuilder fbb;
  fbb.Finish(CreatePotAndAbsolutePosition(
      fbb, 0.0, ::std::numeric_limits<double>::quiet_NaN(), 0.0));
  const auto sensor_values =
      flatbuffers::GetRoot<PotAndAbsolutePosition>(fbb.GetBufferPointer());
  for (size_t i = 0; i < kSampleSize - 1; ++i) {
    estimator.UpdateEstimate(*sensor_values);
  }
  ASSERT_FALSE(estimator.error());

  estimator.UpdateEstimate(*sensor_values);
  ASSERT_TRUE(estimator.error());

  flatbuffers::FlatBufferBuilder fbb2;
  fbb2.Finish(estimator.GetEstimatorState(&fbb2));

  const PotAndAbsoluteEncoderEstimatorState *state =
      flatbuffers::GetRoot<PotAndAbsoluteEncoderEstimatorState>(
          fbb2.GetBufferPointer());

  EXPECT_THAT(*state->errors(),
              ::testing::ElementsAre(ZeroingError::LOST_ABSOLUTE_ENCODER));
}

}  // namespace testing
}  // namespace zeroing
}  // namespace frc971
