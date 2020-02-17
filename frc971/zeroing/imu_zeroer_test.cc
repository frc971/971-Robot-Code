#include "aos/flatbuffers.h"
#include "gtest/gtest.h"
#include "frc971/zeroing/imu_zeroer.h"

namespace frc971::zeroing {

aos::FlatbufferDetachedBuffer<IMUValues> MakeMeasurement(
    const Eigen::Vector3d &gyro, const Eigen::Vector3d &accel) {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  IMUValuesBuilder builder(fbb);
  builder.add_gyro_x(gyro.x());
  builder.add_gyro_y(gyro.y());
  builder.add_gyro_z(gyro.z());
  builder.add_accelerometer_x(accel.x());
  builder.add_accelerometer_y(accel.y());
  builder.add_accelerometer_z(accel.z());
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
  zeroer.ProcessMeasurement(MakeMeasurement({1, 2, 3}, {4, 5, 6}).message());
  ASSERT_FALSE(zeroer.Zeroed());
  ASSERT_FALSE(zeroer.Faulted());
  ASSERT_EQ(0.0, zeroer.GyroOffset().norm());
  ASSERT_EQ(1.0, zeroer.ZeroedGyro().x());
  ASSERT_EQ(2.0, zeroer.ZeroedGyro().y());
  ASSERT_EQ(3.0, zeroer.ZeroedGyro().z());
  ASSERT_EQ(4.0, zeroer.ZeroedAccel().x());
  ASSERT_EQ(5.0, zeroer.ZeroedAccel().y());
  ASSERT_EQ(6.0, zeroer.ZeroedAccel().z());
}

// Tests that we zero if we receive a bunch of identical measurements.
TEST(ImuZeroerTest, ZeroOnConstantData) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  for (size_t ii = 0; ii < ImuZeroer::kSamplesToAverage; ++ii) {
    ASSERT_FALSE(zeroer.Zeroed());
    zeroer.ProcessMeasurement(MakeMeasurement({1, 2, 3}, {4, 5, 6}).message());
  }
  ASSERT_TRUE(zeroer.Zeroed());
  ASSERT_FALSE(zeroer.Faulted());
  // Gyro should be zeroed to {1, 2, 3}.
  ASSERT_EQ(1.0, zeroer.GyroOffset().x());
  ASSERT_EQ(2.0, zeroer.GyroOffset().y());
  ASSERT_EQ(3.0, zeroer.GyroOffset().z());
  ASSERT_EQ(0.0, zeroer.ZeroedGyro().x());
  ASSERT_EQ(0.0, zeroer.ZeroedGyro().y());
  ASSERT_EQ(0.0, zeroer.ZeroedGyro().z());
  // Accelerometer readings should not be affected.
  ASSERT_EQ(4.0, zeroer.ZeroedAccel().x());
  ASSERT_EQ(5.0, zeroer.ZeroedAccel().y());
  ASSERT_EQ(6.0, zeroer.ZeroedAccel().z());
  // If we get another measurement offset by {1, 1, 1} we should read the result
  // as {1, 1, 1}.
  zeroer.ProcessMeasurement(MakeMeasurement({2, 3, 4}, {0, 0, 0}).message());
  ASSERT_FALSE(zeroer.Faulted());
  ASSERT_EQ(1.0, zeroer.ZeroedGyro().x());
  ASSERT_EQ(1.0, zeroer.ZeroedGyro().y());
  ASSERT_EQ(1.0, zeroer.ZeroedGyro().z());
}

// Tests that we tolerate small amounts of noise in the incoming data and can
// still zero.
TEST(ImuZeroerTest, ZeroOnLowNoiseData) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  for (size_t ii = 0; ii < ImuZeroer::kSamplesToAverage; ++ii) {
    ASSERT_FALSE(zeroer.Zeroed());
    const double offset =
        (static_cast<double>(ii) / (ImuZeroer::kSamplesToAverage - 1) - 0.5) *
        0.01;
    zeroer.ProcessMeasurement(
        MakeMeasurement({1 + offset, 2 + offset, 3 + offset},
                        {4 + offset, 5 + offset, 6 + offset})
            .message());
  }
  ASSERT_TRUE(zeroer.Zeroed());
  ASSERT_FALSE(zeroer.Faulted());
  // Gyro should be zeroed to {1, 2, 3}.
  ASSERT_NEAR(1.0, zeroer.GyroOffset().x(), 1e-8);
  ASSERT_NEAR(2.0, zeroer.GyroOffset().y(), 1e-8);
  ASSERT_NEAR(3.0, zeroer.GyroOffset().z(), 1e-8);
  // If we get another measurement offset by {1, 1, 1} we should read the result
  // as {1, 1, 1}.
  zeroer.ProcessMeasurement(MakeMeasurement({2, 3, 4}, {0, 0, 0}).message());
  ASSERT_FALSE(zeroer.Faulted());
  ASSERT_NEAR(1.0, zeroer.ZeroedGyro().x(), 1e-8);
  ASSERT_NEAR(1.0, zeroer.ZeroedGyro().y(), 1e-8);
  ASSERT_NEAR(1.0, zeroer.ZeroedGyro().z(), 1e-8);
  ASSERT_EQ(0.0, zeroer.ZeroedAccel().x());
  ASSERT_EQ(0.0, zeroer.ZeroedAccel().y());
  ASSERT_EQ(0.0, zeroer.ZeroedAccel().z());
}

// Tests that we do not zero if there is too much noise in the input data.
TEST(ImuZeroerTest, NoZeroOnHighNoiseData) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  for (size_t ii = 0; ii < ImuZeroer::kSamplesToAverage; ++ii) {
    ASSERT_FALSE(zeroer.Zeroed());
    const double offset =
        (static_cast<double>(ii) / (ImuZeroer::kSamplesToAverage - 1) - 0.5) *
        1.0;
    zeroer.ProcessMeasurement(
        MakeMeasurement({1 + offset, 2 + offset, 3 + offset},
                        {4 + offset, 5 + offset, 6 + offset})
            .message());
  }
  ASSERT_FALSE(zeroer.Zeroed());
  ASSERT_FALSE(zeroer.Faulted());
}

// Tests that we fault if we successfully rezero and get a significantly offset
// zero.
TEST(ImuZeroerTest, FaultOnNewZero) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  for (size_t ii = 0; ii < ImuZeroer::kSamplesToAverage; ++ii) {
    ASSERT_FALSE(zeroer.Zeroed());
    zeroer.ProcessMeasurement(MakeMeasurement({1, 2, 3}, {4, 5, 6}).message());
  }
  ASSERT_TRUE(zeroer.Zeroed());
  for (size_t ii = 0; ii < ImuZeroer::kSamplesToAverage; ++ii) {
    ASSERT_FALSE(zeroer.Faulted())
        << "We should not fault until we complete a second cycle of zeroing.";
    zeroer.ProcessMeasurement(MakeMeasurement({1, 5, 3}, {4, 5, 6}).message());
  }
  ASSERT_TRUE(zeroer.Faulted());
}

// Tests that we do not fault if the zero only changes by a small amount.
TEST(ImuZeroerTest, NoFaultOnSimilarZero) {
  ImuZeroer zeroer;
  ASSERT_FALSE(zeroer.Zeroed());
  for (size_t ii = 0; ii < ImuZeroer::kSamplesToAverage; ++ii) {
    ASSERT_FALSE(zeroer.Zeroed());
    zeroer.ProcessMeasurement(MakeMeasurement({1, 2, 3}, {4, 5, 6}).message());
  }
  ASSERT_TRUE(zeroer.Zeroed());
  for (size_t ii = 0; ii < ImuZeroer::kSamplesToAverage; ++ii) {
    zeroer.ProcessMeasurement(
        MakeMeasurement({1, 2.0001, 3}, {4, 5, 6}).message());
  }
  ASSERT_FALSE(zeroer.Faulted());
}

}  // namespace frc971::zeroing
