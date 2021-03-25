#include "aos/flatbuffers.h"
#include "gtest/gtest.h"
#include "frc971/zeroing/imu_zeroer.h"

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

// Tests that when we initialize everything is in a sane state.
TEST(ImuZeroerTest, InitializeUnzeroed) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  ASSERT_FALSE(zeroer.Faulted());
  ASSERT_EQ(0.0, zeroer.GyroOffset().norm());
  ASSERT_EQ(0.0, zeroer.ZeroedGyro().norm());
  ASSERT_EQ(0.0, zeroer.ZeroedAccel().norm());
  // A measurement before we are zeroed should just result in the measurement
  // being passed through without modification.
  zeroer.InsertAndProcessMeasurement(
      MakeMeasurement({0.01, 0.02, 0.03}, {4, 5, 6}).message());
  ASSERT_FALSE(zeroer.Zeroed());
  ASSERT_FALSE(zeroer.Faulted());
  ASSERT_EQ(0.0, zeroer.GyroOffset().norm());
  ASSERT_FLOAT_EQ(0.01, zeroer.ZeroedGyro().x());
  ASSERT_FLOAT_EQ(0.02, zeroer.ZeroedGyro().y());
  ASSERT_FLOAT_EQ(0.03, zeroer.ZeroedGyro().z());
  ASSERT_EQ(4.0, zeroer.ZeroedAccel().x());
  ASSERT_EQ(5.0, zeroer.ZeroedAccel().y());
  ASSERT_EQ(6.0, zeroer.ZeroedAccel().z());
}

// Tests that we zero if we receive a bunch of identical measurements.
TEST(ImuZeroerTest, ZeroOnConstantData) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  for (size_t ii = 0; ii < kMinSamplesToZero; ++ii) {
    zeroer.InsertAndProcessMeasurement(
        MakeMeasurement({0.01, 0.02, 0.03}, {4, 5, 6}).message());
  }
  ASSERT_TRUE(zeroer.Zeroed());
  ASSERT_FALSE(zeroer.Faulted());
  // Gyro should be zeroed to {1, 2, 3}.
  ASSERT_FLOAT_EQ(0.01, zeroer.GyroOffset().x());
  ASSERT_FLOAT_EQ(0.02, zeroer.GyroOffset().y());
  ASSERT_FLOAT_EQ(0.03, zeroer.GyroOffset().z());
  ASSERT_EQ(0.0, zeroer.ZeroedGyro().x());
  ASSERT_EQ(0.0, zeroer.ZeroedGyro().y());
  ASSERT_EQ(0.0, zeroer.ZeroedGyro().z());
  // Accelerometer readings should not be affected.
  ASSERT_EQ(4.0, zeroer.ZeroedAccel().x());
  ASSERT_EQ(5.0, zeroer.ZeroedAccel().y());
  ASSERT_EQ(6.0, zeroer.ZeroedAccel().z());
  // If we get another measurement offset by {1, 1, 1} we should read the result
  // as {1, 1, 1}.
  zeroer.InsertAndProcessMeasurement(
      MakeMeasurement({0.02, 0.03, 0.04}, {0, 0, 0}).message());
  ASSERT_FALSE(zeroer.Faulted());
  ASSERT_FLOAT_EQ(0.01, zeroer.ZeroedGyro().x());
  ASSERT_FLOAT_EQ(0.01, zeroer.ZeroedGyro().y());
  ASSERT_FLOAT_EQ(0.01, zeroer.ZeroedGyro().z());
}

// Tests that we do not zero if the gyro is producing particularly high
// magnitude results.
TEST(ImuZeroerTest, NoZeroOnHighMagnitudeGyro) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  for (size_t ii = 0; ii < kMinSamplesToZero; ++ii) {
    zeroer.InsertAndProcessMeasurement(
        MakeMeasurement({0.1, 0.2, 0.3}, {4, 5, 6}).message());
    ASSERT_FALSE(zeroer.Zeroed());
  }
  ASSERT_FALSE(zeroer.Faulted());
}

// Tests that we tolerate small amounts of noise in the incoming data and can
// still zero.
TEST(ImuZeroerTest, ZeroOnLowNoiseData) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  for (size_t ii = 0; ii < kMinSamplesToZero; ++ii) {
    const double offset =
        (static_cast<double>(ii) / (kMinSamplesToZero - 1) - 0.5) * 0.001;
    zeroer.InsertAndProcessMeasurement(
        MakeMeasurement({0.01 + offset, 0.02 + offset, 0.03 + offset},
                        {4 + offset, 5 + offset, 6 + offset}).message());
  }
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
  ASSERT_NEAR(0.01, zeroer.ZeroedGyro().x(), 1e-3);
  ASSERT_NEAR(0.01, zeroer.ZeroedGyro().y(), 1e-3);
  ASSERT_NEAR(0.01, zeroer.ZeroedGyro().z(), 1e-3);
  ASSERT_EQ(0.0, zeroer.ZeroedAccel().x());
  ASSERT_EQ(0.0, zeroer.ZeroedAccel().y());
  ASSERT_EQ(0.0, zeroer.ZeroedAccel().z());
}

// Tests that we do not zero if there is too much noise in the input data.
TEST(ImuZeroerTest, NoZeroOnHighNoiseData) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  for (size_t ii = 0; ii < kMinSamplesToZero; ++ii) {
    ASSERT_FALSE(zeroer.Zeroed());
    const double offset =
        (static_cast<double>(ii) / (kMinSamplesToZero - 1) - 0.5) * 1.0;
    zeroer.InsertAndProcessMeasurement(
        MakeMeasurement({0.01 + offset, 0.02 + offset, 0.03 + offset},
                        {4 + offset, 5 + offset, 6 + offset}).message());
  }
  ASSERT_FALSE(zeroer.Zeroed());
  ASSERT_FALSE(zeroer.Faulted());
}

// Tests that we fault if we successfully rezero and get a significantly offset
// zero.
TEST(ImuZeroerTest, FaultOnNewZero) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  for (size_t ii = 0; ii < kMinSamplesToZero; ++ii) {
    zeroer.InsertAndProcessMeasurement(
        MakeMeasurement({0.01, 0.02, 0.03}, {4, 5, 6}).message());
  }
  ASSERT_TRUE(zeroer.Zeroed());
  ASSERT_FALSE(zeroer.Faulted())
      << "We should not fault until we complete a second cycle of zeroing.";
  for (size_t ii = 0; ii < kMinSamplesToZero; ++ii) {
    zeroer.InsertAndProcessMeasurement(
        MakeMeasurement({0.01, 0.05, 0.03}, {4, 5, 6}).message());
  }
  ASSERT_TRUE(zeroer.Faulted());
  ASSERT_FALSE(zeroer.Zeroed());
}

// Tests that we do not fault if the zero only changes by a small amount.
TEST(ImuZeroerTest, NoFaultOnSimilarZero) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  for (size_t ii = 0; ii < kMinSamplesToZero; ++ii) {
    zeroer.InsertAndProcessMeasurement(
        MakeMeasurement({0.01, 0.02, 0.03}, {4, 5, 6}).message());
  }
  ASSERT_TRUE(zeroer.Zeroed());
  for (size_t ii = 0; ii < kMinSamplesToZero; ++ii) {
    zeroer.InsertAndProcessMeasurement(
        MakeMeasurement({0.01, 0.020001, 0.03}, {4, 5, 6}).message());
  }
  ASSERT_FALSE(zeroer.Faulted());
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
