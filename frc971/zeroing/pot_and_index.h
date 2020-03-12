#ifndef FRC971_ZEROING_POT_AND_INDEX_H_
#define FRC971_ZEROING_POT_AND_INDEX_H_

#include <vector>

#include "flatbuffers/flatbuffers.h"

#include "frc971/zeroing/zeroing.h"

namespace frc971 {
namespace zeroing {

// Estimates the position with an incremental encoder with an index pulse and a
// potentiometer.
class PotAndIndexPulseZeroingEstimator
    : public ZeroingEstimator<PotAndIndexPosition,
                              constants::PotAndIndexPulseZeroingConstants,
                              EstimatorState> {
 public:
  explicit PotAndIndexPulseZeroingEstimator(
      const constants::PotAndIndexPulseZeroingConstants &constants);

  // Update the internal logic with the next sensor values.
  void UpdateEstimate(const PotAndIndexPosition &info) override;

  // Reset the internal logic so it needs to be re-zeroed.
  void Reset() override;

  // Manually trigger an internal error. This is used for testing the error
  // logic.
  void TriggerError() override;

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
  bool offset_ready() const override {
    return start_pos_samples_.size() == constants_.average_filter_size;
  }

  // Returns information about our current state.
  virtual flatbuffers::Offset<State> GetEstimatorState(
      flatbuffers::FlatBufferBuilder *fbb) const override;

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

}  // namespace zeroing
}  // namespace frc971

#endif  // FRC971_ZEROING_POT_AND_INDEX_H_
