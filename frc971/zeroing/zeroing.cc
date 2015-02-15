#include "frc971/zeroing/zeroing.h"

#include <math.h>
#include <vector>

namespace frc971 {
namespace zeroing {

ZeroingEstimator::ZeroingEstimator(
    const constants::Values::ZeroingConstants& constants) {
  index_diff_ = constants.index_difference;
  max_sample_count_ = constants.average_filter_size;
  index_pulse_count_after_reset_ = 0;

  start_pos_samples_.reserve(max_sample_count_);

  Reset();
}

void ZeroingEstimator::Reset() {
  samples_idx_ = 0;
  start_pos_ = 0;
  start_pos_samples_.clear();
  zeroed_ = false;
  wait_for_index_pulse_ = true;
}

void ZeroingEstimator::UpdateEstimate(const PotAndIndexPosition& info) {
  // We want to make sure that we encounter at least one index pulse while
  // zeroing. So we take the index pulse count from the first sample after
  // reset and wait for that count to change before we consider ourselves
  // zeroed.
  if (wait_for_index_pulse_) {
    index_pulse_count_after_reset_ = info.index_pulses;
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
  if (info.index_pulses == index_pulse_count_after_reset_ ||
      offset_ratio_ready() < 1.0) {
    start_pos_ = start_average;
  } else {
    // We calculate an aproximation of the value of the last index position.
    double index_pos = start_average + info.latched_encoder;
    // We round index_pos to the closest valid value of the index.
    double accurate_index_pos = (round(index_pos / index_diff_)) * index_diff_;
    // Now we reverse the first calculation to the accurate start position.
    start_pos_ = accurate_index_pos - info.latched_encoder;

    // Now that we have an accurate starting position we can consider ourselves
    // zeroed.
    zeroed_ = true;
  }

  pos_ = start_pos_ + info.encoder;
}

}  // namespace zeroing
}  // namespace frc971
