#include "frc971/zeroing/imu_zeroer.h"

namespace frc971::zeroing {

ImuZeroer::ImuZeroer() {
  gyro_average_.setZero();
  last_gyro_sample_.setZero();
  last_accel_sample_.setZero();
}

bool ImuZeroer::Zeroed() const { return zeroed_ || Faulted(); }

bool ImuZeroer::Faulted() const { return faulted_; }

Eigen::Vector3d ImuZeroer::ZeroedGyro() const {
  return last_gyro_sample_ - gyro_average_;
}
Eigen::Vector3d ImuZeroer::ZeroedAccel() const { return last_accel_sample_; }
Eigen::Vector3d ImuZeroer::GyroOffset() const { return gyro_average_; }

bool ImuZeroer::GyroZeroReady() const {
  return gyro_averager_.full() && gyro_averager_.GetRange() < kGyroMaxVariation;
}

bool ImuZeroer::AccelZeroReady() const {
  return accel_averager_.full() &&
         accel_averager_.GetRange() < kAccelMaxVariation;
}

void ImuZeroer::ProcessMeasurement(const IMUValues &values) {
  last_gyro_sample_ << values.gyro_x(), values.gyro_y(), values.gyro_z();
  gyro_averager_.AddData(last_gyro_sample_);
  last_accel_sample_ << values.accelerometer_x(), values.accelerometer_y(),
                           values.accelerometer_z();
  accel_averager_.AddData(last_accel_sample_);
  if (GyroZeroReady() && AccelZeroReady()) {
    if (!zeroed_) {
      zeroed_ = true;
      gyro_average_ = gyro_averager_.GetAverage();
    } else {
      // If we got a new zero and it is substantially different from the
      // original zero, fault.
      if ((gyro_averager_.GetAverage() - gyro_average_).norm() >
          kGyroFaultVariation) {
        faulted_ = true;
      }
    }
    gyro_averager_.Reset();
  }
}

}  // namespace frc971::zeroing
