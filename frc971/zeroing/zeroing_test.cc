#include <unistd.h>

#include <memory>

#include <random>

#include "gtest/gtest.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/zeroing/zeroing.h"
#include "aos/die.h"
#include "frc971/control_loops/position_sensor_sim.h"

namespace frc971 {
namespace zeroing {

using constants::AbsoluteEncoderZeroingConstants;
using constants::EncoderPlusIndexZeroingConstants;
using constants::PotAndAbsoluteEncoderZeroingConstants;
using constants::PotAndIndexPulseZeroingConstants;
using control_loops::PositionSensorSimulator;
using FBB = flatbuffers::FlatBufferBuilder;

static const size_t kSampleSize = 30;
static const double kAcceptableUnzeroedError = 0.2;
static const double kIndexErrorFraction = 0.3;
static const size_t kMovingBufferSize = 3;

class ZeroingTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  void MoveTo(PositionSensorSimulator *simulator,
              PotAndIndexPulseZeroingEstimator *estimator,
              double new_position) {
    simulator->MoveTo(new_position);
    FBB fbb;
    estimator->UpdateEstimate(
        *simulator->FillSensorValues<PotAndIndexPosition>(&fbb));
  }

  void MoveTo(PositionSensorSimulator *simulator,
              AbsoluteEncoderZeroingEstimator *estimator, double new_position) {
    simulator->MoveTo(new_position);
    FBB fbb;
    estimator->UpdateEstimate(
        *simulator->FillSensorValues<AbsolutePosition>(&fbb));
  }

  void MoveTo(PositionSensorSimulator *simulator,
              PotAndAbsoluteEncoderZeroingEstimator *estimator,
              double new_position) {
    simulator->MoveTo(new_position);
    FBB fbb;
    estimator->UpdateEstimate(
        *simulator->FillSensorValues<PotAndAbsolutePosition>(&fbb));
  }

  void MoveTo(PositionSensorSimulator *simulator,
              PulseIndexZeroingEstimator *estimator, double new_position) {
    simulator->MoveTo(new_position);
    FBB fbb;
    estimator->UpdateEstimate(
        *simulator->FillSensorValues<IndexPosition>(&fbb));
  }

  void MoveTo(PositionSensorSimulator *simulator,
              HallEffectAndPositionZeroingEstimator *estimator,
              double new_position) {
    simulator->MoveTo(new_position);
    FBB fbb;
    estimator->UpdateEstimate(
        *simulator->FillSensorValues<HallEffectAndPosition>(&fbb));
  }

  template <typename T>
  double GetEstimatorPosition(T *estimator) {
    FBB fbb;
    fbb.Finish(estimator->GetEstimatorState(&fbb));
    return flatbuffers::GetRoot<typename T::State>(fbb.GetBufferPointer())
        ->position();
  }
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
  ASSERT_NEAR(3.3 * index_diff, GetEstimatorPosition(&estimator),
              kAcceptableUnzeroedError * index_diff);

  for (int i = 0; i < 300; i++) {
    MoveTo(&sim, &estimator, 3.9 * index_diff);
  }
  ASSERT_NEAR(3.9 * index_diff, GetEstimatorPosition(&estimator),
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
  ASSERT_NEAR(3.6, GetEstimatorPosition(&estimator),
              kAcceptableUnzeroedError * index_diff);

  // With a single index pulse the zeroing estimator should be able to lock
  // onto the true value of the position.
  MoveTo(&sim, &estimator, 4.01);
  ASSERT_NEAR(4.01, GetEstimatorPosition(&estimator), 0.001);

  MoveTo(&sim, &estimator, 4.99);
  ASSERT_NEAR(4.99, GetEstimatorPosition(&estimator), 0.001);

  MoveTo(&sim, &estimator, 3.99);
  ASSERT_NEAR(3.99, GetEstimatorPosition(&estimator), 0.001);

  MoveTo(&sim, &estimator, 3.01);
  ASSERT_NEAR(3.01, GetEstimatorPosition(&estimator), 0.001);

  MoveTo(&sim, &estimator, 13.55);
  ASSERT_NEAR(13.55, GetEstimatorPosition(&estimator), 0.001);
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
  ASSERT_NEAR(3.5 * index_diff, GetEstimatorPosition(&estimator),
              kAcceptableUnzeroedError * index_diff);

  // With a single index pulse the zeroing estimator should be able to lock
  // onto the true value of the position.
  MoveTo(&sim, &estimator, 4.01);
  ASSERT_NEAR(4.01, GetEstimatorPosition(&estimator), 0.001);

  MoveTo(&sim, &estimator, 4.99);
  ASSERT_NEAR(4.99, GetEstimatorPosition(&estimator), 0.001);

  MoveTo(&sim, &estimator, 3.99);
  ASSERT_NEAR(3.99, GetEstimatorPosition(&estimator), 0.001);

  MoveTo(&sim, &estimator, 3.01);
  ASSERT_NEAR(3.01, GetEstimatorPosition(&estimator), 0.001);

  MoveTo(&sim, &estimator, 13.55);
  ASSERT_NEAR(13.55, GetEstimatorPosition(&estimator), 0.001);
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
  ASSERT_DOUBLE_EQ(3.7 * index_diff, GetEstimatorPosition(&estimator));

  // Trigger one more index pulse and check the offset.
  MoveTo(&sim, &estimator, 4.7 * index_diff);
  ASSERT_DOUBLE_EQ(3.3 * index_diff, estimator.offset());
  ASSERT_DOUBLE_EQ(4.7 * index_diff, GetEstimatorPosition(&estimator));
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
TEST_F(ZeroingTest, TestPotAndAbsoluteEncoderZeroingWithoutMovement) {
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
TEST_F(ZeroingTest, TestPotAndAbsoluteEncoderZeroingIgnoresNAN) {
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
  FBB fbb;
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
TEST_F(ZeroingTest, TestPotAndAbsoluteEncoderZeroingWithMovement) {
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
TEST_F(ZeroingTest, TestPotAndAbsoluteEncoderZeroingWithNaN) {
  PotAndAbsoluteEncoderZeroingConstants constants{
      kSampleSize, 1, 0.3, 0.1, kMovingBufferSize, kIndexErrorFraction};

  PotAndAbsoluteEncoderZeroingEstimator estimator(constants);

  FBB fbb;
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
}

// Tests that an error is detected when the starting position changes too much.
TEST_F(ZeroingTest, TestRelativeEncoderZeroing) {
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
TEST_F(ZeroingTest, TestRelativeEncoderSlipping) {
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

// Test fixture for HallEffectAndPositionZeroingEstimator.
class HallEffectAndPositionZeroingEstimatorTest : public ZeroingTest {
 public:
  // The starting position of the system.
  static constexpr double kStartPosition = 2.0;

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

  HallEffectAndPositionZeroingEstimatorTest()
      : constants_(MakeConstants()), sim_(constants_.index_difference) {
    // Start the system out at the starting position.
    sim_.InitializeHallEffectAndPosition(kStartPosition,
                                         constants_.lower_hall_position,
                                         constants_.upper_hall_position);
  }

 protected:
  // Constants, and the simulation using them.
  const constants::HallEffectZeroingConstants constants_;
  PositionSensorSimulator sim_;
};

// Tests that an error is detected when the starting position changes too much.
TEST_F(HallEffectAndPositionZeroingEstimatorTest, TestHallEffectZeroing) {
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
TEST_F(HallEffectAndPositionZeroingEstimatorTest, TestTooShortPulse) {
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
TEST_F(HallEffectAndPositionZeroingEstimatorTest, TestWrongDirectionNoZero) {
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
TEST_F(HallEffectAndPositionZeroingEstimatorTest, TestStartingOnNoZero) {
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

// Makes sure that using an absolute encoder lets us zero without moving.
TEST_F(ZeroingTest, TestAbsoluteEncoderZeroingWithoutMovement) {
  const double index_diff = 1.0;
  PositionSensorSimulator sim(index_diff);

  const double kMiddlePosition = 2.5;
  const double start_pos = 2.1;
  double measured_absolute_position = 0.3 * index_diff;

  AbsoluteEncoderZeroingConstants constants{
      kSampleSize,        index_diff, measured_absolute_position,
      kMiddlePosition,    0.1,        kMovingBufferSize,
      kIndexErrorFraction};

  sim.Initialize(start_pos, index_diff / 3.0, 0.0,
                 constants.measured_absolute_position);

  AbsoluteEncoderZeroingEstimator estimator(constants);

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
TEST_F(ZeroingTest, TestAbsoluteEncoderZeroingIgnoresNAN) {
  const double index_diff = 1.0;
  PositionSensorSimulator sim(index_diff);

  const double start_pos = 2.1;
  double measured_absolute_position = 0.3 * index_diff;
  const double kMiddlePosition = 2.5;

  AbsoluteEncoderZeroingConstants constants{
      kSampleSize,        index_diff, measured_absolute_position,
      kMiddlePosition,    0.1,        kMovingBufferSize,
      kIndexErrorFraction};

  sim.Initialize(start_pos, index_diff / 3.0, 0.0,
                 constants.measured_absolute_position);

  AbsoluteEncoderZeroingEstimator estimator(constants);

  // We tolerate a couple NANs before we start.
  FBB fbb;
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
  EXPECT_DOUBLE_EQ(start_pos, estimator.offset());
}

// Makes sure that using an absolute encoder doesn't let us zero while moving.
TEST_F(ZeroingTest, TestAbsoluteEncoderZeroingWithMovement) {
  const double index_diff = 1.0;
  PositionSensorSimulator sim(index_diff);

  const double start_pos = 10 * index_diff;
  double measured_absolute_position = 0.3 * index_diff;
  const double kMiddlePosition = 2.5;

  AbsoluteEncoderZeroingConstants constants{
      kSampleSize,        index_diff, measured_absolute_position,
      kMiddlePosition,    0.1,        kMovingBufferSize,
      kIndexErrorFraction};

  sim.Initialize(start_pos, index_diff / 3.0, 0.0,
                 constants.measured_absolute_position);

  AbsoluteEncoderZeroingEstimator estimator(constants);

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
  AbsoluteEncoderZeroingConstants constants{
      kSampleSize, 1, 0.3, 1.0, 0.1, kMovingBufferSize, kIndexErrorFraction};

  AbsoluteEncoderZeroingEstimator estimator(constants);

  FBB fbb;
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
}

}  // namespace zeroing
}  // namespace frc971
