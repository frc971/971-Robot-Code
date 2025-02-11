#include "frc971/zeroing/imu_zeroer.h"

#include "gtest/gtest.h"

#include "aos/flatbuffers.h"

namespace frc971::zeroing {

static constexpr int kMinSamplesToZero =
    2 * ImuZeroer::kSamplesToAverage * ImuZeroer::kRequiredZeroPoints;

aos::FlatbufferDetachedBuffer<IMUValues> MakeMeasurement(
    const Eigen::Vector3d &gyro, const Eigen::Vector3d &accel,
    bool faulted = false) {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);

  ADIS16470DiagStatBuilder diag_stat_builder(fbb);
  diag_stat_builder.add_clock_error(faulted);
  diag_stat_builder.add_memory_failure(false);
  diag_stat_builder.add_sensor_failure(false);
  diag_stat_builder.add_standby_mode(false);
  diag_stat_builder.add_spi_communication_error(false);
  diag_stat_builder.add_flash_memory_update_error(false);
  diag_stat_builder.add_data_path_overrun(false);

  const auto diag_stat_offset = diag_stat_builder.Finish();

  IMUValuesBuilder builder(fbb);
  builder.add_gyro_x(gyro.x());
  builder.add_gyro_y(gyro.y());
  builder.add_gyro_z(gyro.z());
  builder.add_accelerometer_x(accel.x());
  builder.add_accelerometer_y(accel.y());
  builder.add_accelerometer_z(accel.z());
  builder.add_previous_reading_diag_stat(diag_stat_offset);
  fbb.Finish(builder.Finish());
  return fbb.Release();
}

/**
 * @brief Adds noisy IMU measurements to the zeroer.
 *
 * Varies gyro and accel values linearly from -range/2 to +range/2 over
 * a set number of iterations. Each measurement is sent to the zeroer using
 * MakeMeasurement. Used because our zeroer intentionally faults on receiving
 * exactly identical data too many times in a row.
 *
 * @param zeroer ImuZeroer to process measurements.
 * @param gyro Base gyroscope measurement.
 * @param accel Base accelerometer measurement.
 * @param range Range of noise variation.
 * @param faulted Marks measurements as faulted.
 * @param iterations Number of measurements to make.
 */
void MakeNoisyMeasurements(ImuZeroer &zeroer, const Eigen::Vector3d &gyro,
                           const Eigen::Vector3d &accel, double range,
                           bool faulted = false,
                           size_t iterations = kMinSamplesToZero) {
  for (size_t ii = 0; ii < iterations; ++ii) {
    const double offset =
        (static_cast<double>(ii) / (iterations - 1) - 0.5) * range;
    zeroer.InsertAndProcessMeasurement(
        MakeMeasurement(
            {gyro.x() + offset, gyro.y() + offset, gyro.z() + offset},
            {accel.x() + offset, accel.y() + offset, accel.z() + offset},
            faulted)
            .message());
  }
}

// Tests that when we initialize everything is in a sane state.
TEST(ImuZeroerTest, InitializeUnzeroed) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  ASSERT_FALSE(zeroer.Faulted());
  ASSERT_EQ(0.0, zeroer.GyroOffset().norm());
  ASSERT_EQ(0.0, zeroer.ZeroedGyro().value().norm());
  ASSERT_EQ(0.0, zeroer.ZeroedAccel().value().norm());
  // A measurement before we are zeroed should just result in the measurement
  // being passed through without modification.
  zeroer.InsertAndProcessMeasurement(
      MakeMeasurement({0.01, 0.02, 0.03}, {4, 5, 6}).message());
  ASSERT_FALSE(zeroer.Zeroed());
  ASSERT_FALSE(zeroer.Faulted());
  ASSERT_EQ(0.0, zeroer.GyroOffset().norm());
  ASSERT_FLOAT_EQ(0.01, zeroer.ZeroedGyro().value().x());
  ASSERT_FLOAT_EQ(0.02, zeroer.ZeroedGyro().value().y());
  ASSERT_FLOAT_EQ(0.03, zeroer.ZeroedGyro().value().z());
  ASSERT_EQ(4.0, zeroer.ZeroedAccel().value().x());
  ASSERT_EQ(5.0, zeroer.ZeroedAccel().value().y());
  ASSERT_EQ(6.0, zeroer.ZeroedAccel().value().z());
}

// Tests that we zero if we receive a bunch of nearly identical measurements.
// We add a small amount of noise because our zeroer intentionally faults on
// receiving exactly identical data too many times in a row.
TEST(ImuZeroerTest, ZeroOnNearlyConstantData) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  MakeNoisyMeasurements(zeroer, {0.01, 0.02, 0.03}, {4, 5, 6}, 0.0001);
  ASSERT_TRUE(zeroer.Zeroed());
  ASSERT_FALSE(zeroer.Faulted());
  // Gyro should be zeroed to {1, 2, 3}.
  constexpr double kEps = 0.0001;
  ASSERT_NEAR(0.01, zeroer.GyroOffset().x(), kEps);
  ASSERT_NEAR(0.02, zeroer.GyroOffset().y(), kEps);
  ASSERT_NEAR(0.03, zeroer.GyroOffset().z(), kEps);
  ASSERT_NEAR(0.0, zeroer.ZeroedGyro().value().x(), kEps);
  ASSERT_NEAR(0.0, zeroer.ZeroedGyro().value().y(), kEps);
  ASSERT_NEAR(0.0, zeroer.ZeroedGyro().value().z(), kEps);
  // Accelerometer readings should not be affected.
  ASSERT_NEAR(4.0, zeroer.ZeroedAccel().value().x(), kEps);
  ASSERT_NEAR(5.0, zeroer.ZeroedAccel().value().y(), kEps);
  ASSERT_NEAR(6.0, zeroer.ZeroedAccel().value().z(), kEps);
  // If we get another measurement offset by {1, 1, 1} we should read the result
  // as {1, 1, 1}.
  zeroer.InsertAndProcessMeasurement(
      MakeMeasurement({0.02, 0.03, 0.04}, {0, 0, 0}).message());
  ASSERT_FALSE(zeroer.Faulted());
  ASSERT_NEAR(0.01, zeroer.ZeroedGyro().value().x(), kEps);
  ASSERT_NEAR(0.01, zeroer.ZeroedGyro().value().y(), kEps);
  ASSERT_NEAR(0.01, zeroer.ZeroedGyro().value().z(), kEps);
}

// Tests that we do not zero if the gyro is producing particularly high
// magnitude results.
TEST(ImuZeroerTest, NoZeroOnHighMagnitudeGyro) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  for (size_t ii = 0; ii < kMinSamplesToZero; ++ii) {
    const double offset =
        (static_cast<double>(ii) / (kMinSamplesToZero - 1) - 0.5) * 0.0001;
    zeroer.InsertAndProcessMeasurement(
        MakeMeasurement({0.1 + offset, 0.2 + offset, 0.3 + offset},
                        {4 + offset, 5 + offset, 6 + offset})
            .message());
    ASSERT_FALSE(zeroer.Zeroed());
  }
  ASSERT_FALSE(zeroer.Faulted());
}

// Tests that we tolerate small amounts of noise in the incoming data and can
// still zero.
TEST(ImuZeroerTest, ZeroOnLowNoiseData) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  MakeNoisyMeasurements(zeroer, {0.01, 0.02, 0.03}, {4, 5, 6}, 0.001);
  ASSERT_TRUE(zeroer.Zeroed());
  ASSERT_FALSE(zeroer.Faulted());
  ASSERT_NEAR(0.01, zeroer.GyroOffset().x(), 1e-3);
  ASSERT_NEAR(0.02, zeroer.GyroOffset().y(), 1e-3);
  ASSERT_NEAR(0.03, zeroer.GyroOffset().z(), 1e-3);
  // If we get another measurement offset by {0.01, 0.01, 0.01} we should read
  // the result as {0.01, 0.01, 0.01}.
  zeroer.InsertAndProcessMeasurement(
      MakeMeasurement({0.02, 0.03, 0.04}, {0, 0, 0}).message());
  ASSERT_FALSE(zeroer.Faulted());
  ASSERT_NEAR(0.01, zeroer.ZeroedGyro().value().x(), 1e-3);
  ASSERT_NEAR(0.01, zeroer.ZeroedGyro().value().y(), 1e-3);
  ASSERT_NEAR(0.01, zeroer.ZeroedGyro().value().z(), 1e-3);
  ASSERT_EQ(0.0, zeroer.ZeroedAccel().value().x());
  ASSERT_EQ(0.0, zeroer.ZeroedAccel().value().y());
  ASSERT_EQ(0.0, zeroer.ZeroedAccel().value().z());
}

// Tests that we do not zero if there is too much noise in the input data.
TEST(ImuZeroerTest, NoZeroOnHighNoiseData) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  MakeNoisyMeasurements(zeroer, {0.01, 0.02, 0.03}, {4, 5, 6}, 1.0);
  ASSERT_FALSE(zeroer.Zeroed());
  ASSERT_FALSE(zeroer.Faulted());
}

// Tests that we fault if we successfully rezero and get a significantly offset
// zero.
TEST(ImuZeroerTest, FaultOnNewZero) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  MakeNoisyMeasurements(zeroer, {0.01, 0.02, 0.03}, {4, 5, 6}, 0.0001);
  ASSERT_TRUE(zeroer.Zeroed());
  ASSERT_FALSE(zeroer.Faulted())
      << "We should not fault until we complete a second cycle of zeroing.";

  for (size_t ii = 0; ii < kMinSamplesToZero; ++ii) {
    const double offset =
        (static_cast<double>(ii) / (kMinSamplesToZero - 1) - 0.5) * 0.0001;
    zeroer.InsertAndProcessMeasurement(
        MakeMeasurement({0.01 + offset, -0.05 + offset, 0.03 + offset},
                        {4 + offset, 5 + offset, 6 + offset})
            .message());

    if (ii ==
        ImuZeroer::kSamplesToAverage + ImuZeroer::kSamplesToAverage / 40) {
      ASSERT_TRUE(zeroer.Faulted());
      ASSERT_FALSE(zeroer.Zeroed());
    }
  }
  ASSERT_FALSE(zeroer.Faulted());
  ASSERT_TRUE(zeroer.Zeroed());
}

// Tests that we do not fault if the zero only changes by a small amount.
TEST(ImuZeroerTest, NoFaultOnSimilarZero) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  MakeNoisyMeasurements(zeroer, {0.01, 0.02, 0.03}, {4, 5, 6}, 0.0001);
  ASSERT_TRUE(zeroer.Zeroed());
  MakeNoisyMeasurements(zeroer, {0.01, 0.0201, 0.03}, {4, 5, 6}, 0.0001);
  ASSERT_FALSE(zeroer.Faulted());
}

TEST(ImuZeroerTest, FaultOnManyIdenticalReadings) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  for (size_t ii = 0; ii < kMinSamplesToZero; ++ii) {
    zeroer.InsertAndProcessMeasurement(
        MakeMeasurement({0.01, 0.02, 0.03}, {4, 5, 6}).message());
  }
  ASSERT_FALSE(zeroer.Zeroed());
  ASSERT_TRUE(zeroer.Faulted());
}

// Tests that we fault on a bad diagnostic.
TEST(ImuZeroerTest, FaultOnBadDiagnostic) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  ASSERT_FALSE(zeroer.Faulted());
  zeroer.InsertAndProcessMeasurement(
      MakeMeasurement({0.01, 0.02, 0.03}, {4, 5, 6}, true).message());
  ASSERT_FALSE(zeroer.Zeroed());
  ASSERT_TRUE(zeroer.Faulted());
}

}  // namespace frc971::zeroing
