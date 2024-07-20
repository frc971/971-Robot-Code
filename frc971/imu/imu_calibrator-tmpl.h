#include "absl/flags/declare.h"
#include "absl/flags/flag.h"

#include "frc971/imu/imu_calibrator.h"
#include "frc971/math/interpolate.h"

ABSL_DECLARE_FLAG(int32_t, imu_zeroing_buffer);

namespace frc971::imu {

inline constexpr double kGravityGs = 1.0;
// rad / sec
inline constexpr double kGyroMaxZeroingValue = 0.1;

template <typename Scalar>
void ImuCalibrator<Scalar>::InsertImu(size_t imu_index,
                                      const RawImuReading &reading) {
  CHECK_LT(imu_index, imu_readings_.size());
  std::vector<ImuReading> &readings = imu_readings_[imu_index];
  if (readings.size() > 0u) {
    CHECK_LT(readings.back().capture_time_raw, reading.capture_time)
        << ": Readings must be inserted in time order per IMU.";
  }
  // Execute the stationary logic. We identify if this reading is plausibly
  // stationary, then if it is not stationary, we go back in time to any
  // potentially relevant readings and mark them as not stationary. Finally, we
  // go through and as values exit the FLAGS_imu_zeroing_buffer window we do any
  // processing that we can given that we know it must be stationary.
  const bool plausibly_stationary =
      reading.gyro.squaredNorm() < kGyroMaxZeroingValue * kGyroMaxZeroingValue;
  bool stationary = plausibly_stationary;
  int earliest_affected_index =
      readings.size() - absl::GetFlag(FLAGS_imu_zeroing_buffer);
  for (size_t index = std::max(0, earliest_affected_index);
       index < readings.size(); ++index) {
    if (!plausibly_stationary) {
      readings[index].stationary = false;
    }
    if (!readings[index].plausibly_stationary) {
      stationary = false;
    }
  }

  // Since we don't have data from before the start, assume that we may have
  // been moving.
  if (earliest_affected_index < 0) {
    stationary = false;
  }

  if (earliest_affected_index >= 0) {
    ImuReading &earliest_reading = readings[earliest_affected_index];
    // The stationary flag for this reading can no longer change, so we can
    // start to do things based on it.
    earliest_reading.stationary_is_locked = true;
    if (earliest_reading.stationary) {
      earliest_reading.parameter_residuals.gravity =
          earliest_reading.accel.norm() - kGravityGs;
      earliest_reading.parameter_residuals.gyro_zero = earliest_reading.gyro;
      LOG(INFO) << earliest_reading.gyro.transpose();
    }
  }

  const ImuConfig<Scalar> &config = imu_configs_[imu_index];
  Scalar capture_time_adjusted =
      static_cast<Scalar>(aos::time::DurationInSeconds(
          reading.capture_time.time_since_epoch())) -
      (config.parameters.has_value() ? config.parameters->time_offset
                                     : static_cast<Scalar>(0.0));

  imu_readings_[imu_index].emplace_back(
      reading.capture_time, capture_time_adjusted,
      reading.gyro - config.dynamic_params.gyro_zero,
      reading.accel / config.dynamic_params.gravity,
      DynamicImuParameters<Scalar>{static_cast<Scalar>(0.0),
                                   Eigen::Matrix<Scalar, 3, 1>::Zero()},
      plausibly_stationary, stationary, false, std::nullopt, std::nullopt);
}

template <typename Scalar>
void ImuCalibrator<Scalar>::EvaluateRelativeResiduals() {
  for (const auto &readings : imu_readings_) {
    CHECK_LT(static_cast<size_t>(absl::GetFlag(FLAGS_imu_zeroing_buffer) * 2),
             readings.size())
        << ": Insufficient readings to perform calibration.";
  }
  Scalar base_clock = imu_readings_[origin_index_][0].capture_time_adjusted;
  // Current index corresponding to the base_clock time.
  std::vector<size_t> reading_indices(imu_configs_.size(), 0);
  // The for loops are set up so that we:
  // 1. Iterate over every pair of readings from the origin/base IMU.
  // 2. For each other IMU, we identify 0 or 1 readings that fall between those
  //    two readings of the origin IMU. We then calculate the residuals for
  //    that IMU relative to the origin IMU, linearly interpolating between
  //    the pair of readings from (1) (by doing a linear interpolation, we can
  //    get sub-cycle accuracy on time offsets).
  for (;
       reading_indices[origin_index_] < imu_readings_[origin_index_].size() - 1;
       ++reading_indices[origin_index_]) {
    const ImuReading &base_reading =
        imu_readings_[origin_index_][reading_indices[origin_index_]];
    const ImuReading &next_base_reading =
        imu_readings_[origin_index_][reading_indices[origin_index_] + 1];
    base_clock = base_reading.capture_time_adjusted;
    const Scalar next_base_clock = next_base_reading.capture_time_adjusted;
    for (size_t imu_index = 0; imu_index < imu_configs_.size(); ++imu_index) {
      const ImuConfig<Scalar> &imu_config = imu_configs_[imu_index];
      // We don't care about calculating the offsets from the origin IMU to
      // itself...
      if (imu_config.is_origin) {
        continue;
      }
      auto &readings = imu_readings_[imu_index];
      bool done_with_imu = false;
      // This will put the index for the current IMU just past the base_clock
      // timepoint, allowing us to interpolate between
      // reading_indices[origin_index_] and reading_indices[origin_index_] + 1.
      while (readings[reading_indices[imu_index]].capture_time_adjusted <
             base_clock) {
        if (reading_indices[imu_index] == imu_readings_[imu_index].size() - 1) {
          done_with_imu = true;
          break;
        }
        ++reading_indices[imu_index];
      }
      // If we've run out of readings on this imu, stop doing anything.
      if (done_with_imu) {
        continue;
      }
      ImuReading &reading = readings[reading_indices[imu_index]];
      const Scalar reading_time = reading.capture_time_adjusted;
      if (reading_time >= next_base_clock) {
        // There is a gap in readings for this imu; we can't meaningfully
        // populate the residuals.
        continue;
      }
      // Sanity check the above logic.
      CHECK_LE(base_clock, reading_time);
      CHECK_LT(reading_time, next_base_clock);
      CHECK(imu_config.parameters.has_value());
      reading.gyro_residual =
          imu_config.parameters.value().rotation * reading.gyro -
          frc971::math::Interpolate<Eigen::Matrix<Scalar, 3, 1>, Scalar>(
              base_clock, next_base_clock, base_reading.gyro,
              next_base_reading.gyro, reading_time);
      if (!reading.stationary_is_locked || !reading.stationary) {
        continue;
      }
      // In order to calculate the accelerometer residual, we are assuming that
      // the two IMUs "should" produce identical accelerations. This is only
      // true when not rotating. Future changes may account for coriolis
      // effects.
      reading.accel_residual =
          imu_config.parameters.value().rotation * reading.accel -
          frc971::math::Interpolate(base_clock, next_base_clock,
                                    base_reading.accel, next_base_reading.accel,
                                    reading_time);
    }
  }
}

// Helpers to accommodate serializing residuals into the ceres buffer. These
// helpers all return a buffer that points to the next value to be populated.
namespace internal {
template <typename Scalar>
std::span<Scalar> SerializeScalar(Scalar value, std::span<Scalar> out) {
  DCHECK(!out.empty());
  out[0] = value;
  return out.subspan(1);
}
template <typename Scalar, int kSize>
std::span<Scalar> SerializeVector(const Eigen::Matrix<Scalar, kSize, 1> &value,
                                  std::span<Scalar> out) {
  DCHECK_LE(static_cast<size_t>(value.size()), out.size());
  for (int index = 0; index < kSize; ++index) {
    out[index] = value(index);
  }
  return out.subspan(kSize);
}
template <typename Scalar>
std::span<Scalar> SerializeParams(const DynamicImuParameters<Scalar> &params,
                                  std::span<Scalar> out) {
  return SerializeVector(params.gyro_zero,
                         SerializeScalar(params.gravity, out));
}
inline constexpr int kResidualsPerReading = 10u;
}  // namespace internal

template <typename Scalar>
void ImuCalibrator<Scalar>::CalculateResiduals(std::span<Scalar> residuals) {
  EvaluateRelativeResiduals();
  for (size_t imu_index = 0; imu_index < imu_configs_.size(); ++imu_index) {
    const auto &readings = imu_readings_[imu_index];
    double valid_gyro_reading_count = 0;
    double valid_accel_reading_count = 0;
    for (size_t reading_index = 0; reading_index < readings.size();
         ++reading_index) {
      const auto &reading = readings[reading_index];
      if (reading.gyro_residual.has_value()) {
        ++valid_gyro_reading_count;
      }
      if (reading.accel_residual.has_value()) {
        ++valid_accel_reading_count;
      }
    }
    if (!imu_configs_[imu_index].is_origin) {
      CHECK_LT(0, valid_gyro_reading_count);
      CHECK_LT(0, valid_accel_reading_count);
    } else {
      valid_gyro_reading_count = readings.size();
      valid_accel_reading_count = readings.size();
    }
    // Adjust the residuals of the readings to ensure that the solver doesn't
    // cheat by just making it so that the time-offsets are completely
    // misaligned and we can say that all the residuals are "zero".
    const Scalar gyro_reading_scalar =
        static_cast<Scalar>(readings.size() / valid_gyro_reading_count);
    const Scalar accel_reading_scalar =
        static_cast<Scalar>(readings.size() / valid_accel_reading_count);
    for (size_t reading_index = 0; reading_index < readings.size();
         ++reading_index) {
      const auto &reading = readings[reading_index];
      const Scalar *const start_residual = residuals.data();
      // 4 residuals (gravity scalar; gyro zeroes)
      residuals =
          internal::SerializeParams(reading.parameter_residuals, residuals);
      const Eigen::Matrix<Scalar, 3, 1> gyro_residual =
          reading.gyro_residual.value_or(Eigen::Matrix<Scalar, 3, 1>::Zero()) *
          gyro_reading_scalar;
      // 3 residuals
      residuals = internal::SerializeVector(gyro_residual, residuals);
      const Eigen::Matrix<Scalar, 3, 1> accel_residual =
          reading.accel_residual.value_or(Eigen::Matrix<Scalar, 3, 1>::Zero()) *
          accel_reading_scalar;
      // 3 residuals
      residuals = internal::SerializeVector(accel_residual, residuals);
      CHECK_EQ(internal::kResidualsPerReading,
               residuals.data() - start_residual)
          << ": Need to update kResidualsPerReading.";
    }
  }
}

template <typename Scalar>
size_t ImuCalibrator<Scalar>::CalculateNumResiduals(
    const std::vector<size_t> &num_readings) {
  size_t num_residuals = 0;
  for (const size_t count : num_readings) {
    num_residuals += internal::kResidualsPerReading * count;
  }
  return num_residuals;
}

}  // namespace frc971::imu
