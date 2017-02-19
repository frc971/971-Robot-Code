#ifndef FRC971_ZEROING_ZEROING_H_
#define FRC971_ZEROING_ZEROING_H_

#include <cstdint>
#include <vector>

#include "frc971/control_loops/control_loops.q.h"
#include "frc971/constants.h"

// TODO(pschrader): Flag an error if encoder index pulse is not n revolutions
// away from the last one (i.e. got extra counts from noise, etc..)
//
// TODO(pschrader): Flag error if the pot disagrees too much with the encoder
// after being zeroed.
//
// TODO(pschrader): Watch the offset over long periods of time and flag if it
// gets too far away from the initial value.

namespace frc971 {
namespace zeroing {

// Estimates the position with an incremental encoder with an index pulse and a
// potentiometer.
class PotAndIndexPulseZeroingEstimator {
 public:
  using Position = PotAndIndexPosition;
  using ZeroingConstants = constants::PotAndIndexPulseZeroingConstants;
  using State = EstimatorState;

  PotAndIndexPulseZeroingEstimator(
      const constants::PotAndIndexPulseZeroingConstants &constants);

  // Update the internal logic with the next sensor values.
  void UpdateEstimate(const PotAndIndexPosition &info);

  // Reset the internal logic so it needs to be re-zeroed.
  void Reset();

  // Manually trigger an internal error. This is used for testing the error
  // logic.
  void TriggerError();

  // Returns true if an error has occurred, false otherwise. This gets reset to
  // false when the Reset() function is called.
  bool error() const { return error_; }

  // Returns true if the logic considers the corresponding mechanism to be
  // zeroed. It return false otherwise.
  bool zeroed() const { return zeroed_; }

  // Return the estimated position of the corresponding mechanism. This value
  // is in SI units. For example, the estimator for the elevator would return a
  // value in meters for the height relative to absolute zero.
  double position() const { return position_; }

  // Return the estimated starting position of the corresponding mechansim. In
  // some contexts we refer to this as the "offset".
  double offset() const { return start_pos_; }

  // Return the estimated position of the corresponding mechanism not using the
  // index pulse, even if one is available.
  double filtered_position() const { return filtered_position_; }

  // Returns a number between 0 and 1 that represents the percentage of the
  // samples being used in the moving average filter. A value of 0.0 means that
  // no samples are being used. A value of 1.0 means that the filter is using
  // as many samples as it has room for. For example, after a Reset() this
  // value returns 0.0. As more samples get added with UpdateEstimate(...) the
  // return value starts increasing to 1.0.
  double offset_ratio_ready() const {
    return start_pos_samples_.size() /
           static_cast<double>(constants_.average_filter_size);
  }

  // Returns true if the sample buffer is full.
  bool offset_ready() const {
    return start_pos_samples_.size() == constants_.average_filter_size;
  }

 private:
  // This function calculates the start position given the internal state and
  // the provided `latched_encoder' value.
  double CalculateStartPosition(double start_average,
                                double latched_encoder) const;

  // The zeroing constants used to describe the configuration of the system.
  const constants::PotAndIndexPulseZeroingConstants constants_;

  // The estimated position.
  double position_;
  // The unzeroed filtered position.
  double filtered_position_ = 0.0;
  // The next position in 'start_pos_samples_' to be used to store the next
  // sample.
  int samples_idx_;
  // Last 'max_sample_count_' samples for start positions.
  std::vector<double> start_pos_samples_;
  // The estimated starting position of the mechanism. We also call this the
  // 'offset' in some contexts.
  double start_pos_;
  // Flag for triggering logic that takes note of the current index pulse count
  // after a reset. See `last_used_index_pulse_count_'.
  bool wait_for_index_pulse_;
  // After a reset we keep track of the index pulse count with this. Only after
  // the index pulse count changes (i.e. increments at least once or wraps
  // around) will we consider the mechanism zeroed. We also use this to store
  // the most recent `PotAndIndexPosition::index_pulses' value when the start
  // position was calculated. It helps us calculate the start position only on
  // index pulses to reject corrupted intermediate data.
  uint32_t last_used_index_pulse_count_;
  // Marker to track whether we're fully zeroed yet or not.
  bool zeroed_;
  // Marker to track whether an error has occurred. This gets reset to false
  // whenever Reset() is called.
  bool error_;
  // Stores the position "start_pos" variable the first time the program
  // is zeroed.
  double first_start_pos_;
};

// Estimates the position with an absolute encoder which also reports
// incremental counts, and a potentiometer.
class PotAndAbsEncoderZeroingEstimator {
 public:
  using Position = PotAndAbsolutePosition;
  using ZeroingConstants = constants::PotAndAbsoluteEncoderZeroingConstants;
  using State = AbsoluteEstimatorState;

  PotAndAbsEncoderZeroingEstimator(
      const constants::PotAndAbsoluteEncoderZeroingConstants &constants);

  // Resets the internal logic so it needs to be re-zeroed.
  void Reset();

  // Updates the sensor values for the zeroing logic.
  void UpdateEstimate(const PotAndAbsolutePosition &info);

  // Returns true if the mechanism is zeroed, and false if it isn't.
  bool zeroed() const { return zeroed_; }

  // Return the estimated position of the corresponding mechanism. This value
  // is in SI units. For example, the estimator for the elevator would return a
  // value in meters for the height relative to absolute zero.
  double position() const { return position_; }

  // Return the estimated starting position of the corresponding mechansim. In
  // some contexts we refer to this as the "offset".
  double offset() const { return offset_; }

  // Returns true if an error has occurred, false otherwise. This gets reset to
  // false when the Reset() function is called.
  // TODO(austin): Actually implement this.
  bool error() const { return false; }

  // Returns true if the sample buffer is full.
  bool offset_ready() const {
    return relative_to_absolute_offset_samples_.size() ==
               constants_.average_filter_size &&
           offset_samples_.size() == constants_.average_filter_size;
  }

  // Return the estimated position of the corresponding mechanism not using the
  // index pulse, even if one is available.
  double filtered_position() const { return filtered_position_; }

 private:
  // The zeroing constants used to describe the configuration of the system.
  const constants::PotAndAbsoluteEncoderZeroingConstants constants_;
  // True if the mechanism is zeroed.
  bool zeroed_;
  // Samples of the offset needed to line the relative encoder up with the
  // absolute encoder.
  ::std::vector<double> relative_to_absolute_offset_samples_;
  // Offset between the Pot and Relative encoder position.
  ::std::vector<double> offset_samples_;
  // Last moving_buffer_size position samples to be used to determine if the
  // robot is moving.
  ::std::vector<PotAndAbsolutePosition> buffered_samples_;
  // Pointer to front of the buffered samples.
  int buffered_samples_idx_ = 0;
  // Estimated offset between the pot and relative encoder.
  double pot_relative_encoder_offset_ = 0;
  // Estimated start position of the mechanism
  double offset_ = 0;
  // The next position in 'relative_to_absolute_offset_samples_' and
  // 'encoder_samples_' to be used to store the next sample.
  int samples_idx_;
  // The unzeroed filtered position.
  double filtered_position_ = 0.0;
  // The filtered position.
  double position_ = 0.0;
};

// Populates an EstimatorState struct with information from the zeroing
// estimator.
void PopulateEstimatorState(const PotAndIndexPulseZeroingEstimator &estimator,
                            EstimatorState *state);

void PopulateEstimatorState(const PotAndAbsEncoderZeroingEstimator &estimator,
                            AbsoluteEstimatorState *state);

}  // namespace zeroing
}  // namespace frc971

#endif  // FRC971_ZEROING_ZEROING_H_
