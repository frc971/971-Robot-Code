#include "frc971/zeroing/imu_zeroer.h"

namespace frc971::zeroing {

namespace {
bool DiagStatHasFaults(const ADIS16470DiagStat &diag) {
  return diag.clock_error() || diag.memory_failure() || diag.sensor_failure() ||
         diag.standby_mode() || diag.spi_communication_error() ||
         diag.flash_memory_update_error() || diag.data_path_overrun();
}
bool ReadingHasFaults(const IMUValues &values) {
  if (values.has_previous_reading_diag_stat() &&
      DiagStatHasFaults(*values.previous_reading_diag_stat())) {
    return true;
  }
  if (values.has_self_test_diag_stat() &&
      DiagStatHasFaults(*values.self_test_diag_stat())) {
    return true;
  }
  return false;
}
}  // namespace

ImuZeroer::ImuZeroer() {
  gyro_average_.setZero();
  accel_average_.setZero();
  last_gyro_sample_.setZero();
  last_accel_sample_.setZero();
}

bool ImuZeroer::Zeroed() const {
  return num_zeroes_ > kRequiredZeroPoints && !Faulted();
}

bool ImuZeroer::Faulted() const { return faulted_; }

Eigen::Vector3d ImuZeroer::ZeroedGyro() const {
  return last_gyro_sample_ - gyro_average_;
}
Eigen::Vector3d ImuZeroer::ZeroedAccel() const {
  return last_accel_sample_ - accel_average_;
}
Eigen::Vector3d ImuZeroer::GyroOffset() const { return gyro_average_; }

bool ImuZeroer::GyroZeroReady() const {
  return gyro_averager_.full() &&
         gyro_averager_.GetRange() < kGyroMaxVariation &&
         (last_gyro_sample_.lpNorm<Eigen::Infinity>() <
          kGyroMaxZeroingMagnitude);
}

bool ImuZeroer::AccelZeroReady() const {
  return accel_averager_.full() &&
         accel_averager_.GetRange() < kAccelMaxVariation;
}

void ImuZeroer::InsertAndProcessMeasurement(const IMUValues &values) {
  InsertMeasurement(values);
  ProcessMeasurements();
}

void ImuZeroer::InsertMeasurement(const IMUValues &values) {
  if (ReadingHasFaults(values)) {
    faulted_ = true;
    return;
  }
  last_gyro_sample_ << values.gyro_x(), values.gyro_y(), values.gyro_z();
  gyro_averager_.AddData(last_gyro_sample_);
  last_accel_sample_ << values.accelerometer_x(), values.accelerometer_y(),
                           values.accelerometer_z();
  accel_averager_.AddData(last_accel_sample_);
}

void ImuZeroer::ProcessMeasurements() {
  if (GyroZeroReady() && AccelZeroReady()) {
    ++good_iters_;
    if (good_iters_ > kSamplesToAverage / 40) {
      const Eigen::Vector3d current_gyro_average = gyro_averager_.GetAverage();
      constexpr double kAverageUpdateWeight = 0.05;
      if (num_zeroes_ > 0) {
        gyro_average_ +=
            (current_gyro_average - gyro_average_) * kAverageUpdateWeight;
      } else {
        gyro_average_ = current_gyro_average;
      }
      if (num_zeroes_ > 0) {
        // If we got a new zero and it is substantially different from the
        // original zero, fault.
        if ((current_gyro_average - gyro_average_).norm() >
            kGyroFaultVariation) {
          faulted_ = true;
        }
      }
      ++num_zeroes_;
      gyro_averager_.Reset();
    }
  } else {
    good_iters_ = 0;
  }
}

flatbuffers::Offset<control_loops::drivetrain::ImuZeroerState>
ImuZeroer::PopulateStatus(flatbuffers::FlatBufferBuilder *fbb) const {
  control_loops::drivetrain::ImuZeroerState::Builder builder(*fbb);

  builder.add_zeroed(Zeroed());
  builder.add_faulted(Faulted());
  builder.add_number_of_zeroes(num_zeroes_);

  builder.add_gyro_x_average(GyroOffset().x());
  builder.add_gyro_y_average(GyroOffset().y());
  builder.add_gyro_z_average(GyroOffset().z());

  return builder.Finish();
}

}  // namespace frc971::zeroing
