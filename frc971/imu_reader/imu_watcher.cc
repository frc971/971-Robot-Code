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
        callback,
    TimestampSource timestamp_source)
    : dt_config_(dt_config),
      callback_(std::move(callback)),
      zeroer_(zeroing::ImuZeroer::FaultBehavior::kTemporary),
      left_encoder_(
          -EncoderWrapDistance(drivetrain_distance_per_encoder_tick) / 2.0,
          EncoderWrapDistance(drivetrain_distance_per_encoder_tick)),
      right_encoder_(
          -EncoderWrapDistance(drivetrain_distance_per_encoder_tick) / 2.0,
          EncoderWrapDistance(drivetrain_distance_per_encoder_tick)) {
  event_loop->MakeWatcher("/localizer", [this, timestamp_source, event_loop](
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
      int messages_dropped_this_cycle = 0;
      if (first_valid_data_counter_.has_value()) {
        total_imu_messages_received_++;
        // Only update when we have good checksums, since the data counter
        // could get corrupted.
        if (!zeroer_.Faulted()) {
          if (value->data_counter() < last_data_counter_) {
            data_counter_offset_ += 1 << 16;
          }
          const int total_dropped =
              (1 + value->data_counter() + data_counter_offset_ -
               first_valid_data_counter_.value()) -
              total_imu_messages_received_;
          messages_dropped_this_cycle =
              total_dropped - imu_fault_tracker_.missed_messages;
          imu_fault_tracker_.missed_messages = total_dropped;
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
        const aos::monotonic_clock::time_point pi_read_timestamp =
            aos::monotonic_clock::time_point(
                std::chrono::nanoseconds(value->monotonic_timestamp_ns()));
        // If we can't trust the imu reading, just naively increment the
        // pico timestamp.
        const aos::monotonic_clock::time_point pico_timestamp =
            timestamp_source == TimestampSource::kPi
                ? pi_read_timestamp
                : (zeroer_.Faulted()
                       ? (last_pico_timestamp_.has_value()
                              ? last_pico_timestamp_.value() + kNominalDt
                              : aos::monotonic_clock::epoch())
                       : aos::monotonic_clock::time_point(
                             std::chrono::microseconds(
                                 value->pico_timestamp_us())));
        // TODO(james): If we get large enough drift off of the pico,
        // actually do something about it.
        if (!pico_offset_.has_value()) {
          pico_offset_ = pi_read_timestamp - pico_timestamp;
          last_pico_timestamp_ = pico_timestamp;
        }
        // The pico will sleep for a minimum of 3 seconds when resetting the
        // IMU. Any absence of messages for less than that time is probably not
        // an actual reset.
        if (pico_timestamp < last_pico_timestamp_) {
          pico_offset_.value() += std::chrono::microseconds(1ULL << 32);
        }
        const aos::monotonic_clock::time_point sample_timestamp =
            pico_offset_.value() + pico_timestamp;
        pico_offset_error_ = pi_read_timestamp - sample_timestamp;

        if (last_imu_message_.has_value()) {
          constexpr std::chrono::seconds kMinimumImuResetTime(3);
          const std::chrono::nanoseconds message_time_gap =
              last_imu_message_.value() -
              event_loop->context().monotonic_event_time;
          if (message_time_gap > kMinimumImuResetTime) {
            bool assigned_fault = false;
            // The IMU has probably reset; attempt to determine whether just the
            // IMU was reset or if the IMU and pico reset.
            if (std::chrono::abs(pico_offset_error_) >
                std::chrono::seconds(1)) {
              // If we have this big of a gap, then everything downstream of
              // this is going to complain anyways, so reset the offset.
              pico_offset_.reset();

              imu_fault_tracker_.probable_pico_reset_count++;
              assigned_fault = true;
            }
            // See if we dropped the "right" number of messages for the gap in
            // question.
            if (std::chrono::abs(messages_dropped_this_cycle * kNominalDt -
                                 message_time_gap) >
                std::chrono::milliseconds(100)) {
              imu_fault_tracker_.probable_imu_reset_count++;
              assigned_fault = true;
            }

            if (!assigned_fault) {
              imu_fault_tracker_.unassignable_reset_count++;
            }
          }
        }
        const bool zeroed = zeroer_.Zeroed();

        // When not zeroed, we aim to approximate zero acceleration by doing a
        // zero-order hold on the gyro and setting the accelerometer readings to
        // gravity.
        callback_(sample_timestamp, pi_read_timestamp, encoders,
                  zeroed ? zeroer_.ZeroedGyro().value() : last_gyro_,
                  zeroed ? zeroer_.ZeroedAccel().value()
                         : dt_config_.imu_transform.transpose() *
                               Eigen::Vector3d::UnitZ());

        if (zeroed) {
          last_gyro_ = zeroer_.ZeroedGyro().value();
        }
        last_pico_timestamp_ = pico_timestamp;
      }
      last_imu_message_ = event_loop->context().monotonic_event_time;
    }
  });
}
}  // namespace frc971::controls
