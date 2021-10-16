#ifndef FRC971_ZEROING_IMU_ZEROER_H_
#define FRC971_ZEROING_IMU_ZEROER_H_

#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/wpilib/imu_generated.h"
#include "frc971/zeroing/averager.h"

namespace frc971::zeroing {

// This class handles processing IMU measurements and zeroing them once it is
// able to do so.
class ImuZeroer {
 public:
  // Average 0.5 seconds of data (assuming 2kHz sampling rate).
  // TODO(james): Make the gyro zero in a constant amount of time, rather than a
  // constant number of samples...
  // TODO(james): Run average and GetRange calculations over every sample on
  // every timestep, to provide consistent timing.
  static constexpr size_t kSamplesToAverage = 200;
  static constexpr size_t kRequiredZeroPoints = 10;

  ImuZeroer();
  bool Zeroed() const;
  bool Faulted() const;
  void InsertMeasurement(const IMUValues &values);
  // PErforms the heavier-duty processing for managing zeroing.
  void ProcessMeasurements();
  void InsertAndProcessMeasurement(const IMUValues &values);
  Eigen::Vector3d GyroOffset() const;
  Eigen::Vector3d ZeroedGyro() const;
  Eigen::Vector3d ZeroedAccel() const;

  flatbuffers::Offset<control_loops::drivetrain::ImuZeroerState> PopulateStatus(
      flatbuffers::FlatBufferBuilder *fbb) const;

 private:
  // Max variation (difference between the maximum and minimum value) in a
  // kSamplesToAverage range before we allow using the samples for zeroing.
  // These values are currently based on looking at results from the ADIS16448.
  static constexpr double kGyroMaxVariation = 0.02;        // rad / sec
  // Maximum magnitude we allow the gyro zero to have--this is used to prevent
  // us from zeroing the gyro if we just happen to be spinning at a very
  // consistent non-zero rate. Currently this is only plausible in simulation.
  static constexpr double kGyroMaxZeroingMagnitude = 0.1;  // rad / sec
  // Max variation in the range before we consider the accelerometer readings to
  // be steady.
  static constexpr double kAccelMaxVariation = 0.05;    // g's
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
  Eigen::Vector3d accel_average_;
  Eigen::Vector3d last_gyro_sample_;
  Eigen::Vector3d last_accel_sample_;
  // Whether the zeroing has faulted at any point thus far.
  bool faulted_ = false;
  size_t good_iters_ = 0;
  size_t num_zeroes_ = 0;
};

}  // namespace frc971::zeroing
#endif  // FRC971_ZEROING_IMU_ZEROER_H_
