#include "frc971/zeroing/zeroing.h"
#include <math.h>
#include <vector>

namespace frc971 {
namespace zeroing {

ZeroingEstimator::ZeroingEstimator(double index_difference,
                                   size_t max_sample_count) {
  index_diff_ = index_difference;
  samples_idx_ = 0;
  max_sample_count_ = max_sample_count;
  start_pos_samples_.reserve(max_sample_count);
}

void ZeroingEstimator::UpdateEstimate(const ZeroingInfo & info) {
  if (start_pos_samples_.size() < max_sample_count_) {
    start_pos_samples_.push_back(info.pot - info.encoder);
  } else {
    start_pos_samples_[samples_idx_] = info.pot - info.encoder;
  }
  samples_idx_ = (samples_idx_ + 1) % max_sample_count_;

  double start_average = 0.0;
  for (size_t i = 0; i < start_pos_samples_.size(); ++i) {
    start_average += start_pos_samples_[i];
  }

  // Calculates the average of the starting position.
  start_average = start_average / start_pos_samples_.size();
  /* If the index_encoder is invalid, then we use
   * the average of the starting position to
   * calculate the position.
   */
  if (info.index_count == 0) {
    pos_ = start_average + info.encoder;
  } else {
  	// We calculate an aproximation of the value of the last index position.
    double index_pos = start_average + info.index_encoder;
	// We round index_pos to the closest valid value of the index.
    double accurate_index_pos = (round(index_pos / index_diff_)) * index_diff_;
	// We use accurate_index_pos to calculate the position.
    pos_ = accurate_index_pos + info.encoder - info.index_encoder;
  }
}

double ZeroingEstimator::getPosition() { return pos_; }

}  // namespace zeroing
}  // namespace frc971
