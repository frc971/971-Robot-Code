#include "frc971/zeroing/pot_and_index.h"

#include <cmath>

#include "glog/logging.h"

namespace frc971 {
namespace zeroing {

PotAndIndexPulseZeroingEstimator::PotAndIndexPulseZeroingEstimator(
    const constants::PotAndIndexPulseZeroingConstants &constants)
    : constants_(constants) {
  start_pos_samples_.reserve(constants_.average_filter_size);
  Reset();
}

void PotAndIndexPulseZeroingEstimator::Reset() {
  samples_idx_ = 0;
  offset_ = 0;
  start_pos_samples_.clear();
  zeroed_ = false;
  wait_for_index_pulse_ = true;
  last_used_index_pulse_count_ = 0;
  error_ = false;
}

void PotAndIndexPulseZeroingEstimator::TriggerError() {
  if (!error_) {
    VLOG(1) << "Manually triggered zeroing error.";
    error_ = true;
  }
}

double PotAndIndexPulseZeroingEstimator::CalculateStartPosition(
    double start_average, double latched_encoder) const {
  // We calculate an aproximation of the value of the last index position.
  // Also account for index pulses not lining up with integer multiples of the
  // index_diff.
  double index_pos =
      start_average + latched_encoder - constants_.measured_index_position;
  // We round index_pos to the closest valid value of the index.
  double accurate_index_pos = (round(index_pos / constants_.index_difference)) *
                              constants_.index_difference;
  // Now we reverse the first calculation to get the accurate start position.
  return accurate_index_pos - latched_encoder +
         constants_.measured_index_position;
}

void PotAndIndexPulseZeroingEstimator::UpdateEstimate(
    const PotAndIndexPosition &info) {
  // We want to make sure that we encounter at least one index pulse while
  // zeroing. So we take the index pulse count from the first sample after
  // reset and wait for that count to change before we consider ourselves
  // zeroed.
  if (wait_for_index_pulse_) {
    last_used_index_pulse_count_ = info.index_pulses();
    wait_for_index_pulse_ = false;
  }

  if (start_pos_samples_.size() < constants_.average_filter_size) {
    start_pos_samples_.push_back(info.pot() - info.encoder());
  } else {
    start_pos_samples_[samples_idx_] = info.pot() - info.encoder();
  }

  // Drop the oldest sample when we run this function the next time around.
  samples_idx_ = (samples_idx_ + 1) % constants_.average_filter_size;

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
      (info.index_pulses() == last_used_index_pulse_count_ || !offset_ready())) {
    offset_ = start_average;
  } else if (!zeroed_ || last_used_index_pulse_count_ != info.index_pulses()) {
    // Note the accurate start position and the current index pulse count so
    // that we only run this logic once per index pulse. That should be more
    // resilient to corrupted intermediate data.
    offset_ = CalculateStartPosition(start_average, info.latched_encoder());
    last_used_index_pulse_count_ = info.index_pulses();

    // TODO(austin): Reject encoder positions which have x% error rather than
    // rounding to the closest index pulse.

    // Save the first starting position.
    if (!zeroed_) {
      first_start_pos_ = offset_;
      VLOG(2) << "latching start position" << first_start_pos_;
    }

    // Now that we have an accurate starting position we can consider ourselves
    // zeroed.
    zeroed_ = true;
    // Throw an error if first_start_pos is bigger/smaller than
    // constants_.allowable_encoder_error * index_diff + start_pos.
    if (::std::abs(first_start_pos_ - offset_) >
        constants_.allowable_encoder_error * constants_.index_difference) {
      if (!error_) {
        VLOG(1)
            << "Encoder ticks out of range since last index pulse. first start "
               "position: "
            << first_start_pos_ << " recent starting position: " << offset_
            << ", allowable error: "
            << constants_.allowable_encoder_error * constants_.index_difference;
        error_ = true;
      }
    }
  }

  position_ = offset_ + info.encoder();
  filtered_position_ = start_average + info.encoder();
}

flatbuffers::Offset<PotAndIndexPulseZeroingEstimator::State>
PotAndIndexPulseZeroingEstimator::GetEstimatorState(
    flatbuffers::FlatBufferBuilder *fbb) const {
  State::Builder builder(*fbb);
  builder.add_error(error_);
  builder.add_zeroed(zeroed_);
  builder.add_position(position_);
  builder.add_pot_position(filtered_position_);
  return builder.Finish();
}

}  // namespace zeroing
}  // namespace frc971
