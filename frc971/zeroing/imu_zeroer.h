#ifndef FRC971_ZEROING_IMU_ZEROER_H_
#define FRC971_ZEROING_IMU_ZEROER_H_

#include "frc971/wpilib/imu_generated.h"
#include "frc971/zeroing/averager.h"

namespace frc971::zeroing {

// This class handles processing IMU measurements and zeroing them once it is
// able to do so.
class ImuZeroer {
 public:
  // Average 5 seconds of data (assuming 2kHz sampling rate).
  static constexpr size_t kSamplesToAverage = 10000.0;

  ImuZeroer();
  bool Zeroed() const;
  bool Faulted() const;
  void ProcessMeasurement(const IMUValues &values);
  Eigen::Vector3d GyroOffset() const;
  Eigen::Vector3d ZeroedGyro() const;
  Eigen::Vector3d ZeroedAccel() const;
 private:
  // Max variation (difference between the maximum and minimum value) in a
  // kSamplesToAverage range before we allow using the samples for zeroing.
  // These values are currently based on looking at results from the ADIS16448.
  static constexpr double kGyroMaxVariation = 0.02;     // rad / sec
  // Max variation in the range before we consider the accelerometer readings to
  // be steady.
  static constexpr double kAccelMaxVariation = 0.02;    // g's
  // If we ever are able to rezero and get a zero that is more than
  // kGyroFaultVariation away from the original zeroing, fault.
  static constexpr double kGyroFaultVariation = 0.005;  // rad / sec

  bool GyroZeroReady() const;
  bool AccelZeroReady() const;

  Averager<double, kSamplesToAverage, 3> gyro_averager_;
  // Averager for the accelerometer readings--we don't currently actually
  // average the readings, but we do check that the accelerometer readings have
  // stayed roughly constant during the calibration period.
  Averager<double, kSamplesToAverage, 3> accel_averager_;
  // The average zero position of the gyro.
  Eigen::Vector3d gyro_average_;
  Eigen::Vector3d last_gyro_sample_;
  Eigen::Vector3d last_accel_sample_;
  // Whether we have currently zeroed yet.
  bool zeroed_ = false;
  // Whether the zeroing has faulted at any point thus far.
  bool faulted_ = false;
};

}  // namespace frc971::zeroing
#endif  // FRC971_ZEROING_IMU_ZEROER_H_
