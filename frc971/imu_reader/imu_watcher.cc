#include "frc971/imu_reader/imu_watcher.h"

#include "frc971/wpilib/imu_batch_generated.h"

namespace frc971::controls {
namespace {
// Return the amount of distance that the drivetrain can travel before the
// encoders will wrap. Necessary because the pico only sends over the encoders
// in 16-bit counters, which will wrap relatively readily.
double EncoderWrapDistance(double drivetrain_distance_per_encoder_tick) {
  return drivetrain_distance_per_encoder_tick * (1 << 16);
}
}  // namespace
ImuWatcher::ImuWatcher(
    aos::EventLoop *event_loop,
    const control_loops::drivetrain::DrivetrainConfig<double> &dt_config,
    const double drivetrain_distance_per_encoder_tick,
    std::function<
        void(aos::monotonic_clock::time_point, aos::monotonic_clock::time_point,
             std::optional<Eigen::Vector2d>, Eigen::Vector3d, Eigen::Vector3d)>
        callback)
    : dt_config_(dt_config),
      callback_(std::move(callback)),
      zeroer_(zeroing::ImuZeroer::FaultBehavior::kTemporary),
      left_encoder_(
          -EncoderWrapDistance(drivetrain_distance_per_encoder_tick) / 2.0,
          EncoderWrapDistance(drivetrain_distance_per_encoder_tick)),
      right_encoder_(
          -EncoderWrapDistance(drivetrain_distance_per_encoder_tick) / 2.0,
          EncoderWrapDistance(drivetrain_distance_per_encoder_tick)) {
  event_loop->MakeWatcher("/localizer", [this, event_loop](
                                            const IMUValuesBatch &values) {
    CHECK(values.has_readings());
    for (const IMUValues *value : *values.readings()) {
      zeroer_.InsertAndProcessMeasurement(*value);
      if (zeroer_.Faulted()) {
        if (value->checksum_failed()) {
          imu_fault_tracker_.pico_to_pi_checksum_mismatch++;
        } else if (value->previous_reading_diag_stat()->checksum_mismatch()) {
          imu_fault_tracker_.imu_to_pico_checksum_mismatch++;
        } else {
          imu_fault_tracker_.other_zeroing_faults++;
        }
      } else {
        if (!first_valid_data_counter_.has_value()) {
          first_valid_data_counter_ = value->data_counter();
        }
      }
      if (first_valid_data_counter_.has_value()) {
        total_imu_messages_received_++;
        // Only update when we have good checksums, since the data counter
        // could get corrupted.
        if (!zeroer_.Faulted()) {
          if (value->data_counter() < last_data_counter_) {
            data_counter_offset_ += 1 << 16;
          }
          imu_fault_tracker_.missed_messages =
              (1 + value->data_counter() + data_counter_offset_ -
               first_valid_data_counter_.value()) -
              total_imu_messages_received_;
          last_data_counter_ = value->data_counter();
        }
      }
      // Set encoders to nullopt if we are faulted at all (faults may include
      // checksum mismatches).
      const std::optional<Eigen::Vector2d> encoders =
          zeroer_.Faulted()
              ? std::nullopt
              : std::make_optional(Eigen::Vector2d{
                    left_encoder_.Unwrap(value->left_encoder()),
                    right_encoder_.Unwrap(value->right_encoder())});
      {
        // If we can't trust the imu reading, just naively increment the
        // pico timestamp.
        const aos::monotonic_clock::time_point pico_timestamp =
            zeroer_.Faulted()
                ? (last_pico_timestamp_.has_value()
                       ? last_pico_timestamp_.value() + kNominalDt
                       : aos::monotonic_clock::epoch())
                : aos::monotonic_clock::time_point(
                      std::chrono::microseconds(value->pico_timestamp_us()));
        // TODO(james): If we get large enough drift off of the pico,
        // actually do something about it.
        if (!pico_offset_.has_value()) {
          pico_offset_ =
              event_loop->context().monotonic_event_time - pico_timestamp;
          last_pico_timestamp_ = pico_timestamp;
        }
        if (pico_timestamp < last_pico_timestamp_) {
          pico_offset_.value() += std::chrono::microseconds(1ULL << 32);
        }
        const aos::monotonic_clock::time_point sample_timestamp =
            pico_offset_.value() + pico_timestamp;
        pico_offset_error_ =
            event_loop->context().monotonic_event_time - sample_timestamp;
        const bool zeroed = zeroer_.Zeroed();

        // When not zeroed, we aim to approximate zero acceleration by doing a
        // zero-order hold on the gyro and setting the accelerometer readings to
        // gravity.
        callback_(sample_timestamp,
                  aos::monotonic_clock::time_point(std::chrono::nanoseconds(
                      value->monotonic_timestamp_ns())),
                  encoders, zeroed ? zeroer_.ZeroedGyro().value() : last_gyro_,
                  zeroed ? zeroer_.ZeroedAccel().value()
                         : dt_config_.imu_transform.transpose() *
                               Eigen::Vector3d::UnitZ());

        if (zeroed) {
          last_gyro_ = zeroer_.ZeroedGyro().value();
        }
        last_pico_timestamp_ = pico_timestamp;
      }
    }
  });
}
}  // namespace frc971::controls
