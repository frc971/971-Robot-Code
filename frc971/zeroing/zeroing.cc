#include "frc971/zeroing/zeroing.h"

#include <cmath>
#include <vector>

#include "frc971/zeroing/wrap.h"

namespace frc971 {
namespace zeroing {

void PopulateEstimatorState(
    const zeroing::PotAndIndexPulseZeroingEstimator &estimator,
    EstimatorState *state) {
  state->error = estimator.error();
  state->zeroed = estimator.zeroed();
  state->position = estimator.position();
  state->pot_position = estimator.filtered_position();
}

void PopulateEstimatorState(
    const zeroing::PotAndAbsEncoderZeroingEstimator &estimator,
    AbsoluteEstimatorState *state) {
  state->error = estimator.error();
  state->zeroed = estimator.zeroed();

  state->position = estimator.position();
  state->pot_position = estimator.filtered_position();
}

PotAndIndexPulseZeroingEstimator::PotAndIndexPulseZeroingEstimator(
    const constants::PotAndIndexPulseZeroingConstants &constants)
    : constants_(constants) {
  start_pos_samples_.reserve(constants_.average_filter_size);
  Reset();
}

void PotAndIndexPulseZeroingEstimator::Reset() {
  samples_idx_ = 0;
  start_pos_ = 0;
  start_pos_samples_.clear();
  zeroed_ = false;
  wait_for_index_pulse_ = true;
  last_used_index_pulse_count_ = 0;
  first_start_pos_ = 0.0;
  error_ = false;
}

void PotAndIndexPulseZeroingEstimator::TriggerError() {
  if (!error_) {
    LOG(ERROR, "Manually triggered zeroing error.\n");
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
    last_used_index_pulse_count_ = info.index_pulses;
    wait_for_index_pulse_ = false;
  }

  if (start_pos_samples_.size() < constants_.average_filter_size) {
    start_pos_samples_.push_back(info.pot - info.encoder);
  } else {
    start_pos_samples_[samples_idx_] = info.pot - info.encoder;
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
    // constants_.allowable_encoder_error * index_diff + start_pos.
    if (::std::abs(first_start_pos_ - start_pos_) >
        constants_.allowable_encoder_error * constants_.index_difference) {
      if (!error_) {
        LOG(ERROR,
            "Encoder ticks out of range since last index pulse. first start "
            "position: %f recent starting position: %f, allowable error: %f\n",
            first_start_pos_, start_pos_,
            constants_.allowable_encoder_error * constants_.index_difference);
        error_ = true;
      }
    }
  }

  position_ = start_pos_ + info.encoder;
  filtered_position_ = start_average + info.encoder;
}

PotAndAbsEncoderZeroingEstimator::PotAndAbsEncoderZeroingEstimator(
    const constants::PotAndAbsoluteEncoderZeroingConstants &constants)
    : constants_(constants) {
  relative_to_absolute_offset_samples_.reserve(constants_.average_filter_size);
  offset_samples_.reserve(constants_.average_filter_size);
  Reset();
}

void PotAndAbsEncoderZeroingEstimator::Reset() {
  zeroed_ = false;
  relative_to_absolute_offset_samples_.clear();
  offset_samples_.clear();
}

// So, this needs to be a multistep process.  We need to first estimate the
// offset between the absolute encoder and the relative encoder.  That process
// should get us an absolute number which is off by integer multiples of the
// distance/rev.  In parallel, we can estimate the offset between the pot and
// encoder.  When both estimates have converged, we can then compute the offset
// in a cycle, and which cycle, which gives us the accurate global offset.
//
// It's tricky to compute the offset between the absolute and relative encoder.
// We need to compute this inside 1 revolution.  The easiest way to do this
// would be to wrap the encoder, subtract the two of them, and then average the
// result.  That will struggle when they are off by PI.  Instead, we need to
// wrap the number to +- PI from the current averaged offset.
void PotAndAbsEncoderZeroingEstimator::UpdateEstimate(
    const PotAndAbsolutePosition &info) {
  // TODO(austin): Only add this sample if the robot is stopped.

  // Compute the sum of all the offset samples.
  double relative_to_absolute_offset_sum = 0.0;
  for (size_t i = 0; i < relative_to_absolute_offset_samples_.size(); ++i) {
    relative_to_absolute_offset_sum += relative_to_absolute_offset_samples_[i];
  }

  // Compute the average offset between the absolute encoder and relative
  // encoder.  If we have 0 samples, assume it is 0.
  double average_relative_to_absolute_offset =
      relative_to_absolute_offset_samples_.size() == 0
          ? 0.0
          : relative_to_absolute_offset_sum /
                relative_to_absolute_offset_samples_.size();

  // Now, compute the nearest absolute encoder value to the offset relative
  // encoder position.
  const double adjusted_absolute_encoder =
      Wrap(info.encoder + average_relative_to_absolute_offset,
           info.absolute_encoder - constants_.measured_absolute_position,
           constants_.one_revolution_distance);

  const double relative_to_absolute_offset =
      adjusted_absolute_encoder - info.encoder;

  // Add the sample and update the average with the new reading.
  const size_t relative_to_absolute_offset_samples_size =
      relative_to_absolute_offset_samples_.size();
  if (relative_to_absolute_offset_samples_size <
      constants_.average_filter_size) {
    average_relative_to_absolute_offset =
        (average_relative_to_absolute_offset *
             relative_to_absolute_offset_samples_size +
         relative_to_absolute_offset) /
        (relative_to_absolute_offset_samples_size + 1);

    relative_to_absolute_offset_samples_.push_back(relative_to_absolute_offset);
  } else {
    average_relative_to_absolute_offset -=
        relative_to_absolute_offset_samples_[samples_idx_] /
        relative_to_absolute_offset_samples_size;
    relative_to_absolute_offset_samples_[samples_idx_] =
        relative_to_absolute_offset;
    average_relative_to_absolute_offset +=
        relative_to_absolute_offset / relative_to_absolute_offset_samples_size;
  }

  // Now compute the offset between the pot and relative encoder.
  if (offset_samples_.size() < constants_.average_filter_size) {
    offset_samples_.push_back(info.pot - info.encoder);
  } else {
    offset_samples_[samples_idx_] = info.pot - info.encoder;
  }

  // Drop the oldest sample when we run this function the next time around.
  samples_idx_ = (samples_idx_ + 1) % constants_.average_filter_size;

  double pot_relative_encoder_offset_sum = 0.0;
  for (size_t i = 0; i < offset_samples_.size(); ++i) {
    pot_relative_encoder_offset_sum += offset_samples_[i];
  }
  const double pot_relative_encoder_offset =
      pot_relative_encoder_offset_sum / offset_samples_.size();

  offset_ = Wrap(info.encoder + pot_relative_encoder_offset,
                 average_relative_to_absolute_offset + info.encoder,
                 constants_.one_revolution_distance) -
            info.encoder;
  if (offset_ready()) {
    zeroed_ = true;
  }

  filtered_position_ = pot_relative_encoder_offset + info.encoder;
  position_ = offset_ + info.encoder;
}

}  // namespace zeroing
}  // namespace frc971
