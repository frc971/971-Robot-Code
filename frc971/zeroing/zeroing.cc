#include "frc971/zeroing/zeroing.h"

#include <cmath>
#include <vector>

namespace frc971 {
namespace zeroing {

void PopulateEstimatorState(const zeroing::ZeroingEstimator& estimator,
                            EstimatorState* state) {
  state->error = estimator.error();
  state->zeroed = estimator.zeroed();
  state->position = estimator.position();
  state->pot_position = estimator.filtered_position();
}

ZeroingEstimator::ZeroingEstimator(
    const constants::ZeroingConstants& constants) {
  index_diff_ = constants.index_difference;
  max_sample_count_ = constants.average_filter_size;
  known_index_pos_ = constants.measured_index_position;
  allowable_encoder_error_ = constants.allowable_encoder_error;
  start_pos_samples_.reserve(max_sample_count_);
  Reset();
}

void ZeroingEstimator::Reset() {
  samples_idx_ = 0;
  start_pos_ = 0;
  start_pos_samples_.clear();
  zeroed_ = false;
  wait_for_index_pulse_ = true;
  last_used_index_pulse_count_ = 0;
  first_start_pos_ = 0.0;
  error_ = false;
}

void ZeroingEstimator::TriggerError() {
  if (!error_) {
    LOG(ERROR, "Manually triggered zeroing error.\n");
    error_ = true;
  }
}

double ZeroingEstimator::CalculateStartPosition(double start_average,
                                                double latched_encoder) const {
  // We calculate an aproximation of the value of the last index position.
  // Also account for index pulses not lining up with integer multiples of the
  // index_diff.
  double index_pos = start_average + latched_encoder - known_index_pos_;
  // We round index_pos to the closest valid value of the index.
  double accurate_index_pos = (round(index_pos / index_diff_)) * index_diff_;
  // Now we reverse the first calculation to get the accurate start position.
  return accurate_index_pos - latched_encoder + known_index_pos_;
}

void ZeroingEstimator::UpdateEstimate(const PotAndIndexPosition& info) {
  // We want to make sure that we encounter at least one index pulse while
  // zeroing. So we take the index pulse count from the first sample after
  // reset and wait for that count to change before we consider ourselves
  // zeroed.
  if (wait_for_index_pulse_) {
    last_used_index_pulse_count_ = info.index_pulses;
    wait_for_index_pulse_ = false;
  }

  if (start_pos_samples_.size() < max_sample_count_) {
    start_pos_samples_.push_back(info.pot - info.encoder);
  } else {
    start_pos_samples_[samples_idx_] = info.pot - info.encoder;
  }

  // Drop the oldest sample when we run this function the next time around.
  samples_idx_ = (samples_idx_ + 1) % max_sample_count_;

  double sample_sum = 0.0;

  for (size_t i = 0; i < start_pos_samples_.size(); ++i) {
    sample_sum += start_pos_samples_[i];
  }

  // Calculates the average of the starting position.
  double start_average = sample_sum / start_pos_samples_.size();

  // If there are no index pulses to use or we don't have enough samples yet to
  // have a well-filtered starting position then we use the filtered value as
  // our best guess.
  if (!zeroed_ &&
      (info.index_pulses == last_used_index_pulse_count_ || !offset_ready())) {
    start_pos_ = start_average;
  } else if (!zeroed_ || last_used_index_pulse_count_ != info.index_pulses) {
    // Note the accurate start position and the current index pulse count so
    // that we only run this logic once per index pulse. That should be more
    // resilient to corrupted intermediate data.
    start_pos_ = CalculateStartPosition(start_average, info.latched_encoder);
    last_used_index_pulse_count_ = info.index_pulses;

    // TODO(austin): Reject encoder positions which have x% error rather than
    // rounding to the closest index pulse.

    // Save the first starting position.
    if (!zeroed_) {
      first_start_pos_ = start_pos_;
      LOG(INFO, "latching start position %f\n", first_start_pos_);
    }

    // Now that we have an accurate starting position we can consider ourselves
    // zeroed.
    zeroed_ = true;
    // Throw an error if first_start_pos is bigger/smaller than
    // allowable_encoder_error_ * index_diff + start_pos.
    if (::std::abs(first_start_pos_ - start_pos_) >
        allowable_encoder_error_ * index_diff_) {
      if (!error_) {
        LOG(ERROR,
            "Encoder ticks out of range since last index pulse. first start "
            "position: %f recent starting position: %f, allowable error: %f\n",
            first_start_pos_, start_pos_,
            allowable_encoder_error_ * index_diff_);
        error_ = true;
      }
    }
  }

  pos_ = start_pos_ + info.encoder;
  filtered_position_ = start_average + info.encoder;
}

}  // namespace zeroing
}  // namespace frc971
