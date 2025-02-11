#include "frc971/zeroing/imu_zeroer.h"

ABSL_FLAG(bool, allow_duplicate_samples, false,
          "Allow repeated identical imu measurements?");
ABSL_FLAG(size_t, sample_duplication_threshold, 100,
          "Threshold for the amount of repeated identical imu measurements we "
          "accept before faulting");

namespace frc971::zeroing {

namespace {
bool DiagStatHasFaults(const ADIS16470DiagStat &diag) {
  return diag.clock_error() || diag.memory_failure() || diag.sensor_failure() ||
         diag.standby_mode() || diag.spi_communication_error() ||
         diag.flash_memory_update_error() || diag.data_path_overrun() ||
         diag.checksum_mismatch();
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
  if (values.checksum_failed()) {
    return true;
  }
  return false;
}
}  // namespace

ImuZeroer::ImuZeroer(FaultBehavior fault_behavior, double gyro_max_variation)
    : fault_behavior_(fault_behavior),
      gyro_max_variation_(gyro_max_variation),
      sample_duplication_threshold_(
          absl::GetFlag(FLAGS_sample_duplication_threshold)),
      allow_duplicate_samples_(absl::GetFlag(FLAGS_allow_duplicate_samples)) {
  gyro_average_.setZero();
  last_gyro_sample_.setZero();
  last_accel_sample_.setZero();
}

bool ImuZeroer::Zeroed() const {
  return num_zeroes_ > kRequiredZeroPoints && !Faulted();
}

bool ImuZeroer::Faulted() const { return reading_faulted_ || zeroing_faulted_; }

std::optional<Eigen::Vector3d> ImuZeroer::ZeroedGyro() const {
  return Faulted() ? std::nullopt
                   : std::make_optional<Eigen::Vector3d>(last_gyro_sample_ -
                                                         gyro_average_);
}

std::optional<Eigen::Vector3d> ImuZeroer::ZeroedAccel() const {
  return Faulted() ? std::nullopt
                   : std::make_optional<Eigen::Vector3d>(last_accel_sample_);
}

Eigen::Vector3d ImuZeroer::GyroOffset() const { return gyro_average_; }

bool ImuZeroer::GyroZeroReady() const {
  return gyro_averager_.full() &&
         gyro_averager_.GetRange() < gyro_max_variation_ &&
         (last_gyro_sample_.lpNorm<Eigen::Infinity>() <
          kGyroMaxZeroingMagnitude);
}

bool ImuZeroer::AccelZeroReady() const {
  return accel_averager_.full() &&
         accel_averager_.GetRange() < kAccelMaxVariation;
}

void ImuZeroer::InsertAndProcessMeasurement(const IMUValues &values) {
  if (InsertMeasurement(values)) {
    ProcessMeasurements();
  }
}

bool ImuZeroer::InsertMeasurement(const IMUValues &values) {
  if (ReadingHasFaults(values)) {
    reading_faulted_ = true;
    return false;
  }
  if (fault_behavior_ == FaultBehavior::kTemporary) {
    reading_faulted_ = false;
  }

  last_last_gyro_sample_ = last_gyro_sample_;
  last_last_accel_sample_ = last_accel_sample_;

  last_gyro_sample_ << values.gyro_x(), values.gyro_y(), values.gyro_z();
  gyro_averager_.AddData(last_gyro_sample_);
  last_accel_sample_ << values.accelerometer_x(), values.accelerometer_y(),
      values.accelerometer_z();
  accel_averager_.AddData(last_accel_sample_);

  if (last_last_accel_sample_ == last_accel_sample_ &&
      last_gyro_sample_ == last_last_gyro_sample_) {
    ++sample_duplicate_count_;
  } else {
    sample_duplicate_count_ = 0;
  }

  return true;
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
        // original zero, or if we got too many duplicate readings in a row,
        // fault.
        if (((current_gyro_average - gyro_average_).norm() >
             kGyroFaultVariation) ||
            (sample_duplicate_count_ > sample_duplication_threshold_ &&
             !allow_duplicate_samples_)) {
          zeroing_faulted_ = true;
        } else if (fault_behavior_ == FaultBehavior::kTemporary) {
          zeroing_faulted_ = false;
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

  builder.add_accel_x_average(accel_averager_.GetAverage()[0]);
  builder.add_accel_y_average(accel_averager_.GetAverage()[1]);
  builder.add_accel_z_average(accel_averager_.GetAverage()[2]);

  return builder.Finish();
}

void ImuZeroer::PopulateStatus(
    control_loops::drivetrain::ImuZeroerStateStatic *status) const {
  status->set_zeroed(Zeroed());
  status->set_faulted(Faulted());
  status->set_number_of_zeroes(num_zeroes_);

  status->set_gyro_x_average(GyroOffset().x());
  status->set_gyro_y_average(GyroOffset().y());
  status->set_gyro_z_average(GyroOffset().z());

  status->set_accel_x_average(accel_averager_.GetAverage()[0]);
  status->set_accel_y_average(accel_averager_.GetAverage()[1]);
  status->set_accel_z_average(accel_averager_.GetAverage()[2]);
}

}  // namespace frc971::zeroing
