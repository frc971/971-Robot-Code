#ifndef FRC971_IMU_READER_IMU_WATCHER_H_
#define FRC971_IMU_READER_IMU_WATCHER_H_

#include "aos/events/event_loop.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/imu_reader/imu_failures_generated.h"
#include "frc971/zeroing/imu_zeroer.h"
#include "frc971/zeroing/wrap.h"

namespace frc971::controls {
// This class handles listening to an IMUValuesBatch channel sourced off of our
// ADIS16505 pico board and calling the user-specified callback with the
// relevant data. This intermediary is used to unwrap encoder readings, check
// for checksum mismatches, zero the gyro/accelerometer, and translate
// timestamps between devices.
// TODO(james): Get unit tests for this class specifically written (we already
// have tests for the code that exercises this).
class ImuWatcher {
 public:
  // Expected frequency of messages from the pico-based IMU.
  static constexpr std::chrono::microseconds kNominalDt{500};

  // The callback specified by the user will take:
  // sample_time_pico: The pico-based timestamp corresponding to the measurement
  //   time. This will be offset by roughly pico_offset_error from the pi's
  //   monotonic clock.
  // sample_time_pi: Timestamp from the kernel for when the pi observed the
  //   relevant measurement.
  // encoders: Current encoder values, [left, right]. nullopt if we have faults.
  // gyro: Current gyro readings, in the raw IMU axes (i.e., these must be
  //   rotated by dt_config.imu_transform before being used). Suitable
  //   for input to the down estimator.
  // accel: Current accelerometer readings, in the raw IMU axes (i.e., these
  //   must be rotated by dt_config.imu_transform before being used). Suitable
  //   for input to the down estimator.
  ImuWatcher(
      aos::EventLoop *event_loop,
      const control_loops::drivetrain::DrivetrainConfig<double> &dt_config,
      double drivetrain_distance_per_encoder_tick,
      std::function<void(
          aos::monotonic_clock::time_point, aos::monotonic_clock::time_point,
          std::optional<Eigen::Vector2d>, Eigen::Vector3d, Eigen::Vector3d)>
          callback);

  const zeroing::ImuZeroer &zeroer() const { return zeroer_; }

  flatbuffers::Offset<ImuFailures> PopulateImuFailures(
      flatbuffers::FlatBufferBuilder *fbb) const {
    return ImuFailures::Pack(*fbb, &imu_fault_tracker_);
  }

  // t = pico_offset + pico_timestamp.
  // Note that this can drift over sufficiently long time periods!
  std::optional<std::chrono::nanoseconds> pico_offset() const {
    return pico_offset_;
  }
  // pico_offset_error = actual_time - (pico_offset + pico_timestamp)
  // If the pico clock and pi clock are exactly in sync, this will always be
  // zero.
  aos::monotonic_clock::duration pico_offset_error() const {
    return pico_offset_error_;
  }

 private:
  const control_loops::drivetrain::DrivetrainConfig<double> dt_config_;
  std::function<void(
      aos::monotonic_clock::time_point, aos::monotonic_clock::time_point,
      std::optional<Eigen::Vector2d>, Eigen::Vector3d, Eigen::Vector3d)>
      callback_;

  // Last observed pico measurement. Used to track IMU staleness.
  std::optional<aos::monotonic_clock::time_point> last_pico_timestamp_;
  // Estimate of the drift between the pi and pico clocks. See
  // pico_offset_error() for definition.
  aos::monotonic_clock::duration pico_offset_error_;
  // Raw offset between the pico and pi clocks. Gets updated to compensate for
  // wrapping in the pico timestamp.
  std::optional<std::chrono::nanoseconds> pico_offset_;

  zeroing::ImuZeroer zeroer_;

  ImuFailuresT imu_fault_tracker_;
  // The first observed data counter. This is used to help us track dropped
  // messages.
  std::optional<size_t> first_valid_data_counter_;
  size_t total_imu_messages_received_ = 0;
  // added to the current read data counter to allow the data counter to
  // increase monotonically. Will be a multiple of 2 ** 16.
  size_t data_counter_offset_ = 0;
  // PRevious data counter value (data_counter_offset_ not included).
  int last_data_counter_ = 0;

  // Unwrappers for the left and right encoders (necessary because the pico only
  // sends out 16-bit encoder counts).
  zeroing::UnwrapSensor left_encoder_;
  zeroing::UnwrapSensor right_encoder_;

  // When we lose IMU readings (e.g., due to checksum mismatches), we perform a
  // zero-order hold on gyro readings; in order to do this, store the most
  // recent gyro readings.
  Eigen::Vector3d last_gyro_ = Eigen::Vector3d::Zero();
};
}  // namespace frc971::controls
#endif  // FRC971_IMU_READER_IMU_WATCHER_H_
