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

class ZeroingEstimator {
 public:
  virtual ~ZeroingEstimator(){}

  // Returns true if the logic considers the corresponding mechanism to be
  // zeroed.
  virtual bool zeroed() const = 0;

  // Returns the estimated position of the corresponding mechanism.
  virtual double offset() const = 0;

  // Returns true if there has been an error.
  virtual bool error() const = 0;
};

// Estimates the position with an incremental encoder with an index pulse and a
// potentiometer.
class PotAndIndexPulseZeroingEstimator : public ZeroingEstimator {
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

  bool error() const override { return error_; }

  bool zeroed() const override { return zeroed_; }

  double offset() const override { return offset_; }

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

  // Returns information about our current state.
  State GetEstimatorState() const;

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
  double offset_;
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

// Estimates the position with an incremental encoder with an index pulse and a
// potentiometer.
class HallEffectAndPositionZeroingEstimator : public ZeroingEstimator {
 public:
  using Position = HallEffectAndPosition;
  using ZeroingConstants = constants::HallEffectZeroingConstants;
  using State = HallEffectAndPositionEstimatorState;

  explicit HallEffectAndPositionZeroingEstimator(const ZeroingConstants &constants);

  // Update the internal logic with the next sensor values.
  void UpdateEstimate(const Position &info);

  // Reset the internal logic so it needs to be re-zeroed.
  void Reset();

  // Manually trigger an internal error. This is used for testing the error
  // logic.
  void TriggerError();

  bool error() const override { return error_; }

  bool zeroed() const override { return zeroed_; }

  double offset() const override { return offset_; }

  // Returns information about our current state.
  State GetEstimatorState() const;

 private:
  // Sets the minimum and maximum posedge position values.
  void StoreEncoderMaxAndMin(const HallEffectAndPosition &info);

  // The zeroing constants used to describe the configuration of the system.
  const ZeroingConstants constants_;

  // The estimated state of the hall effect.
  double current_ = 0.0;
  // The estimated position.
  double position_ = 0.0;
  // The smallest and largest positions of the last set of encoder positions
  // while the hall effect was low.
  double min_low_position_;
  double max_low_position_;
  // If we've seen the hall effect high for enough times without going low, then
  // we can be sure it isn't a false positive.
  bool high_long_enough_;
  size_t cycles_high_;

  bool last_hall_ = false;

  // The estimated starting position of the mechanism. We also call this the
  // 'offset' in some contexts.
  double offset_;
  // Flag for triggering logic that takes note of the current posedge count
  // after a reset. See `last_used_posedge_count_'.
  bool initialized_;
  // After a reset we keep track of the posedge count with this. Only after the
  // posedge count changes (i.e. increments at least once or wraps around) will
  // we consider the mechanism zeroed. We also use this to store the most recent
  // `HallEffectAndPosition::posedge_count' value when the start position
  // was calculated. It helps us calculate the start position only on posedges
  // to reject corrupted intermediate data.
  int32_t last_used_posedge_count_;
  // Marker to track whether we're fully zeroed yet or not.
  bool zeroed_;
  // Marker to track whether an error has occurred. This gets reset to false
  // whenever Reset() is called.
  bool error_ = false;
  // Stores the position "start_pos" variable the first time the program
  // is zeroed.
  double first_start_pos_;
};

// Estimates the position with an absolute encoder which also reports
// incremental counts, and a potentiometer.
class PotAndAbsEncoderZeroingEstimator : public ZeroingEstimator {
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

  bool zeroed() const override { return zeroed_; }

  double offset() const override { return offset_; }

  bool error() const override { return error_; }

  // Returns true if the sample buffer is full.
  bool offset_ready() const {
    return relative_to_absolute_offset_samples_.size() ==
               constants_.average_filter_size &&
           offset_samples_.size() == constants_.average_filter_size;
  }

  // Returns information about our current state.
  State GetEstimatorState() const;

 private:
  // The zeroing constants used to describe the configuration of the system.
  const constants::PotAndAbsoluteEncoderZeroingConstants constants_;

  // True if the mechanism is zeroed.
  bool zeroed_;
  // Marker to track whether an error has occurred.
  bool error_;
  // The first valid offset we recorded. This is only set after zeroed_ first
  // changes to true.
  double first_offset_;

  // The filtered absolute encoder.  This is used in the status for calibration.
  double filtered_absolute_encoder_ = 0.0;

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


// Zeros by seeing all the index pulses in the range of motion of the mechanism
// and using that to figure out which index pulse is which.
class PulseIndexZeroingEstimator : public ZeroingEstimator {
 public:
  using Position = IndexPosition;
  using ZeroingConstants = constants::EncoderPlusIndexZeroingConstants;
  using State = IndexEstimatorState;

  PulseIndexZeroingEstimator(
      const constants::EncoderPlusIndexZeroingConstants &constants)
      : constants_(constants) {
    Reset();
  }

  // Resets the internal logic so it needs to be re-zeroed.
  void Reset();

  bool zeroed() const override { return zeroed_; }

  // It's as ready as it'll ever be...
  bool offset_ready() const { return true; }

  double offset() const override { return offset_; }

  bool error() const override { return error_; }

  // Updates the internal logic with the next sensor values.
  void UpdateEstimate(const IndexPosition &info);

  // Returns information about our current state.
  State GetEstimatorState() const;

  void TriggerError() { error_ = true; }

 private:
  // Returns the current real position using the relative encoder offset.
  double CalculateCurrentPosition(const IndexPosition &info);

  // Sets the minimum and maximum index pulse position values.
  void StoreIndexPulseMaxAndMin(const IndexPosition &info);

  // Returns the number of index pulses we should have seen so far.
  int IndexPulseCount() const;

  // Contains the physical constants describing the system.
  const constants::EncoderPlusIndexZeroingConstants constants_;

  // The smallest position of all the index pulses.
  double min_index_position_;
  // The largest position of all the index pulses.
  double max_index_position_;

  // The estimated starting position of the mechanism.
  double offset_;
  // After a reset we keep track of the index pulse count with this. Only after
  // the index pulse count changes (i.e. increments at least once or wraps
  // around) will we consider the mechanism zeroed. We also use this to store
  // the most recent `PotAndIndexPosition::index_pulses' value when the start
  // position was calculated. It helps us calculate the start position only on
  // index pulses to reject corrupted intermediate data.
  uint32_t last_used_index_pulse_count_;

  // True if we are fully zeroed.
  bool zeroed_;
  // Marker to track whether an error has occurred.
  bool error_;

  // The estimated position.
  double position_;
};

}  // namespace zeroing
}  // namespace frc971

#endif  // FRC971_ZEROING_ZEROING_H_
