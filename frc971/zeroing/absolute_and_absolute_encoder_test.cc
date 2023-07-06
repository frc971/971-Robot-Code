#include "frc971/zeroing/absolute_and_absolute_encoder.h"

#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "frc971/zeroing/zeroing_test.h"

namespace frc971 {
namespace zeroing {
namespace testing {

using constants::AbsoluteAndAbsoluteEncoderZeroingConstants;

class AbsoluteAndAbsoluteEncoderZeroingTest : public ZeroingTest {
 protected:
  void MoveTo(PositionSensorSimulator *simulator,
              AbsoluteAndAbsoluteEncoderZeroingEstimator *estimator,
              double new_position) {
    simulator->MoveTo(new_position);
    flatbuffers::FlatBufferBuilder fbb;
    estimator->UpdateEstimate(
        *simulator->FillSensorValues<AbsoluteAndAbsolutePosition>(&fbb));
  }
};

// Makes sure that using an absolute encoder lets us zero without moving.
TEST_F(AbsoluteAndAbsoluteEncoderZeroingTest,
       TestAbsoluteAndAbsoluteEncoderZeroingWithoutMovement) {
  const double full_range = 4.0;

  const double distance_per_revolution = 1.0;
  const double single_turn_distance_per_revolution = full_range;

  const double start_pos = 2.1;

  // Middle position for the single turn absolute encoder.
  const double single_turn_middle_position = full_range * 0.5;

  const double measured_absolute_position = 0.3 * distance_per_revolution;
  const double single_turn_measured_absolute_position =
      0.4 * single_turn_distance_per_revolution;

  AbsoluteAndAbsoluteEncoderZeroingConstants constants{
      kSampleSize,
      distance_per_revolution,
      measured_absolute_position,
      single_turn_distance_per_revolution,
      single_turn_measured_absolute_position,
      single_turn_middle_position,
      0.1,
      kMovingBufferSize,
      kIndexErrorFraction};

  PositionSensorSimulator sim(distance_per_revolution,
                              single_turn_distance_per_revolution);
  sim.Initialize(start_pos, distance_per_revolution / 3.0, 0.0,
                 measured_absolute_position,
                 single_turn_measured_absolute_position);

  AbsoluteAndAbsoluteEncoderZeroingEstimator estimator(constants);

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
TEST_F(AbsoluteAndAbsoluteEncoderZeroingTest,
       TestAbsoluteAndAbsoluteEncoderZeroingIgnoresNAN) {
  const double full_range = 4.0;

  const double distance_per_revolution = 1.0;
  const double single_turn_distance_per_revolution = full_range;

  const double start_pos = 2.1;

  // Middle position for the single turn absolute encoder.
  const double single_turn_middle_position = full_range * 0.5;

  const double measured_absolute_position = 0.3 * distance_per_revolution;
  const double single_turn_measured_absolute_position =
      0.4 * single_turn_distance_per_revolution;

  AbsoluteAndAbsoluteEncoderZeroingConstants constants{
      kSampleSize,
      distance_per_revolution,
      measured_absolute_position,
      single_turn_distance_per_revolution,
      single_turn_measured_absolute_position,
      single_turn_middle_position,
      0.1,
      kMovingBufferSize,
      kIndexErrorFraction};

  PositionSensorSimulator sim(distance_per_revolution,
                              single_turn_distance_per_revolution);
  sim.Initialize(start_pos, distance_per_revolution / 3.0, 0.0,
                 measured_absolute_position,
                 single_turn_measured_absolute_position);

  AbsoluteAndAbsoluteEncoderZeroingEstimator estimator(constants);

  // We tolerate a couple NANs before we start.
  flatbuffers::FlatBufferBuilder fbb;
  fbb.Finish(CreateAbsoluteAndAbsolutePosition(
      fbb, 0.0, ::std::numeric_limits<double>::quiet_NaN(), 0.0));
  for (size_t i = 0; i < kSampleSize - 1; ++i) {
    estimator.UpdateEstimate(*flatbuffers::GetRoot<AbsoluteAndAbsolutePosition>(
        fbb.GetBufferPointer()));
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
TEST_F(AbsoluteAndAbsoluteEncoderZeroingTest,
       TestAbsoluteAndAbsoluteEncoderZeroingWithMovement) {
  const double full_range = 4.0;

  const double distance_per_revolution = 1.0;
  const double single_turn_distance_per_revolution = full_range;

  const double start_pos = 2.1;

  // Middle position for the single turn absolute encoder.
  const double single_turn_middle_position = full_range * 0.5;

  const double measured_absolute_position = 0.3 * distance_per_revolution;
  const double single_turn_measured_absolute_position =
      0.4 * single_turn_distance_per_revolution;

  AbsoluteAndAbsoluteEncoderZeroingConstants constants{
      kSampleSize,
      distance_per_revolution,
      measured_absolute_position,
      single_turn_distance_per_revolution,
      single_turn_measured_absolute_position,
      single_turn_middle_position,
      0.1,
      kMovingBufferSize,
      kIndexErrorFraction};

  PositionSensorSimulator sim(distance_per_revolution,
                              single_turn_distance_per_revolution);
  sim.Initialize(start_pos, distance_per_revolution / 3.0, 0.0,
                 measured_absolute_position,
                 single_turn_measured_absolute_position);

  AbsoluteAndAbsoluteEncoderZeroingEstimator estimator(constants);

  for (size_t i = 0; i < kSampleSize + kMovingBufferSize - 1; ++i) {
    MoveTo(&sim, &estimator, start_pos + i * distance_per_revolution);
    ASSERT_FALSE(estimator.zeroed());
  }
  MoveTo(&sim, &estimator, start_pos + 10 * distance_per_revolution);

  MoveTo(&sim, &estimator, start_pos);
  ASSERT_FALSE(estimator.zeroed());
}

// Makes sure we detect an error if the ZeroingEstimator gets sent a NaN.
TEST_F(AbsoluteAndAbsoluteEncoderZeroingTest,
       TestAbsoluteAndAbsoluteEncoderZeroingWithNaN) {
  AbsoluteAndAbsoluteEncoderZeroingConstants constants{
      kSampleSize,        1, 0.3, 1, 0.3, 2.5, 0.1, kMovingBufferSize,
      kIndexErrorFraction};

  AbsoluteAndAbsoluteEncoderZeroingEstimator estimator(constants);

  flatbuffers::FlatBufferBuilder fbb;
  fbb.Finish(CreateAbsoluteAndAbsolutePosition(
      fbb, 0.0, ::std::numeric_limits<double>::quiet_NaN(), 0.0));
  const auto sensor_values =
      flatbuffers::GetRoot<AbsoluteAndAbsolutePosition>(fbb.GetBufferPointer());
  for (size_t i = 0; i < kSampleSize - 1; ++i) {
    estimator.UpdateEstimate(*sensor_values);
  }
  ASSERT_FALSE(estimator.error());

  estimator.UpdateEstimate(*sensor_values);
  ASSERT_TRUE(estimator.error());

  fbb.Finish(estimator.GetEstimatorState(&fbb));
  const AbsoluteAndAbsoluteEncoderEstimatorState *state =
      flatbuffers::GetRoot<AbsoluteAndAbsoluteEncoderEstimatorState>(
          fbb.GetBufferPointer());

  ASSERT_GT(state->errors()->size(), 0);
  EXPECT_EQ(state->errors()->Get(0), ZeroingError::LOST_ABSOLUTE_ENCODER);
}

TEST_F(AbsoluteAndAbsoluteEncoderZeroingTest,
       TestAbsoluteAndAbsoluteEncoderZeroingState) {
  const double full_range = 4.0;
  const double distance_per_revolution = 1.0;
  const double single_turn_distance_per_revolution = full_range;
  const double start_pos = 2.1;
  const double single_turn_middle_position = full_range * 0.5;
  const double measured_absolute_position = 0.3 * distance_per_revolution;
  const double single_turn_measured_absolute_position =
      0.4 * single_turn_distance_per_revolution;

  AbsoluteAndAbsoluteEncoderZeroingConstants constants{
      kSampleSize,
      distance_per_revolution,
      measured_absolute_position,
      single_turn_distance_per_revolution,
      single_turn_measured_absolute_position,
      single_turn_middle_position,
      0.1,
      kMovingBufferSize,
      kIndexErrorFraction};

  PositionSensorSimulator sim(distance_per_revolution,
                              single_turn_distance_per_revolution);
  sim.Initialize(start_pos, distance_per_revolution / 3.0, 0.0,
                 measured_absolute_position,
                 single_turn_measured_absolute_position);

  AbsoluteAndAbsoluteEncoderZeroingEstimator estimator(constants);

  const double position = 2.7;

  for (size_t i = 0; i < kSampleSize + kMovingBufferSize - 1; ++i) {
    MoveTo(&sim, &estimator, position);
    ASSERT_FALSE(estimator.zeroed());
  }
  MoveTo(&sim, &estimator, position);
  ASSERT_TRUE(estimator.zeroed());
  EXPECT_DOUBLE_EQ(start_pos, estimator.offset());

  flatbuffers::FlatBufferBuilder fbb;

  fbb.Finish(estimator.GetEstimatorState(&fbb));

  const AbsoluteAndAbsoluteEncoderEstimatorState *state =
      flatbuffers::GetRoot<AbsoluteAndAbsoluteEncoderEstimatorState>(
          fbb.GetBufferPointer());

  EXPECT_NEAR(state->position(), position, 1e-10);

  // (position + measured_absolute_position) % distance_per_revolution
  // (2.7 + 0.3) % 1
  EXPECT_NEAR(state->absolute_position(), 0.0, 1e-10);

  // (position + single_turn_measured_absolute_position) %
  // single_turn_distance_per_revolution
  // (2.7 + 1.6) % 4
  EXPECT_NEAR(state->single_turn_absolute_position(), 0.3, 1e-10);
}

// Tests that errors() adds the OFFSET_MOVED_TOO_FAR error when we move too far.
TEST_F(AbsoluteAndAbsoluteEncoderZeroingTest,
       TestAbsoluteAndAbsoluteEncoderZeroingStateErrors) {
  const double full_range = 4.0;
  const double distance_per_revolution = 1.0;
  const double single_turn_distance_per_revolution = full_range;
  const double start_pos = 2.1;
  const double single_turn_middle_position = full_range * 0.5;
  const double measured_absolute_position = 0.3 * distance_per_revolution;
  const double single_turn_measured_absolute_position =
      0.4 * single_turn_distance_per_revolution;

  AbsoluteAndAbsoluteEncoderZeroingConstants constants{
      kSampleSize,
      distance_per_revolution,
      measured_absolute_position,
      single_turn_distance_per_revolution,
      single_turn_measured_absolute_position,
      single_turn_middle_position,
      0.1,
      kMovingBufferSize,
      kIndexErrorFraction};

  PositionSensorSimulator sim(distance_per_revolution,
                              single_turn_distance_per_revolution);
  sim.Initialize(start_pos, distance_per_revolution / 3.0, 0.0,
                 measured_absolute_position,
                 single_turn_measured_absolute_position);

  AbsoluteAndAbsoluteEncoderZeroingEstimator estimator(constants);

  const double position = 2.7;

  for (size_t i = 0; i < kSampleSize + kMovingBufferSize - 1; ++i) {
    MoveTo(&sim, &estimator, position);
    ASSERT_FALSE(estimator.zeroed());
  }
  MoveTo(&sim, &estimator, position);
  ASSERT_TRUE(estimator.zeroed());
  EXPECT_DOUBLE_EQ(start_pos, estimator.offset());

  // If the ratios suddenly get very messed up
  flatbuffers::FlatBufferBuilder fbb;
  fbb.Finish(CreateAbsoluteAndAbsolutePosition(fbb, 0.0, 0.0, 3.0));

  const auto sensor_values =
      flatbuffers::GetRoot<AbsoluteAndAbsolutePosition>(fbb.GetBufferPointer());

  ASSERT_FALSE(estimator.error());

  for (size_t i = 0; i < kSampleSize + kMovingBufferSize - 1; ++i) {
    estimator.UpdateEstimate(*sensor_values);
  }
  ASSERT_TRUE(estimator.error());

  flatbuffers::FlatBufferBuilder fbb2;
  fbb2.Finish(estimator.GetEstimatorState(&fbb2));
  const AbsoluteAndAbsoluteEncoderEstimatorState *state =
      flatbuffers::GetRoot<AbsoluteAndAbsoluteEncoderEstimatorState>(
          fbb2.GetBufferPointer());

  for (ZeroingError err : *state->errors()) {
    LOG(INFO) << "error: " << EnumNameZeroingError(err);
  }
  EXPECT_THAT(*state->errors(),
              ::testing::ElementsAre(ZeroingError::OFFSET_MOVED_TOO_FAR));
}

}  // namespace testing
}  // namespace zeroing
}  // namespace frc971
