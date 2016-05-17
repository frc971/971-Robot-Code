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

// Estimates the position with encoder,
// the pot and the indices.
class ZeroingEstimator {
 public:
  ZeroingEstimator(const constants::ZeroingConstants &constants);

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
  // zeroed. It return false otherwise. For example, right after a call to
  // Reset() this returns false.
  bool zeroed() const { return zeroed_; }

  // Return the estimated position of the corresponding mechanism. This value
  // is in SI units. For example, the estimator for the elevator would return a
  // value in meters for the height relative to absolute zero.
  double position() const { return pos_; }

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
    return start_pos_samples_.size() / static_cast<double>(max_sample_count_);
  }

  // Returns true if the sample buffer is full.
  bool offset_ready() const {
    return start_pos_samples_.size() == max_sample_count_;
  }

 private:
  // This function calculates the start position given the internal state and
  // the provided `latched_encoder' value.
  double CalculateStartPosition(double start_average,
                                double latched_encoder) const;

  // The estimated position.
  double pos_;
  // The unzeroed filtered position.
  double filtered_position_ = 0.0;
  // The distance between two consecutive index positions.
  double index_diff_;
  // The next position in 'start_pos_samples_' to be used to store the next
  // sample.
  int samples_idx_;
  // Last 'max_sample_count_' samples for start positions.
  std::vector<double> start_pos_samples_;
  // The number of the last samples of start position to consider in the
  // estimation.
  size_t max_sample_count_;
  // The estimated starting position of the mechanism. We also call this the
  // 'offset' in some contexts.
  double start_pos_;
  // The absolute position of any index pulse on the mechanism. This is used to
  // account for the various ways the encoders get mounted into the robot.
  double known_index_pos_;
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
  // The fraction of index_diff (possibly greater than 1) after which an error
  // is reported.
  double allowable_encoder_error_;
};

// Populates an EstimatorState struct with information from the zeroing
// estimator.
void PopulateEstimatorState(const ZeroingEstimator &estimator,
                            EstimatorState *state);

}  // namespace zeroing
}  // namespace frc971

#endif  // FRC971_ZEROING_ZEROING_H_
