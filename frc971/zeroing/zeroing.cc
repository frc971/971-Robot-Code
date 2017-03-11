#include "frc971/zeroing/zeroing.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "frc971/zeroing/wrap.h"


namespace frc971 {
namespace zeroing {
namespace {

bool compare_encoder(const PotAndAbsolutePosition &left,
                     const PotAndAbsolutePosition &right) {
  return left.encoder < right.encoder;
}

}  // namespace

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
    offset_ = start_average;
  } else if (!zeroed_ || last_used_index_pulse_count_ != info.index_pulses) {
    // Note the accurate start position and the current index pulse count so
    // that we only run this logic once per index pulse. That should be more
    // resilient to corrupted intermediate data.
    offset_ = CalculateStartPosition(start_average, info.latched_encoder);
    last_used_index_pulse_count_ = info.index_pulses;

    // TODO(austin): Reject encoder positions which have x% error rather than
    // rounding to the closest index pulse.

    // Save the first starting position.
    if (!zeroed_) {
      first_start_pos_ = offset_;
      LOG(INFO, "latching start position %f\n", first_start_pos_);
    }

    // Now that we have an accurate starting position we can consider ourselves
    // zeroed.
    zeroed_ = true;
    // Throw an error if first_start_pos is bigger/smaller than
    // constants_.allowable_encoder_error * index_diff + start_pos.
    if (::std::abs(first_start_pos_ - offset_) >
        constants_.allowable_encoder_error * constants_.index_difference) {
      if (!error_) {
        LOG(ERROR,
            "Encoder ticks out of range since last index pulse. first start "
            "position: %f recent starting position: %f, allowable error: %f\n",
            first_start_pos_, offset_,
            constants_.allowable_encoder_error * constants_.index_difference);
        error_ = true;
      }
    }
  }

  position_ = offset_ + info.encoder;
  filtered_position_ = start_average + info.encoder;
}

PotAndIndexPulseZeroingEstimator::State
PotAndIndexPulseZeroingEstimator::GetEstimatorState() const {
  State r;
  r.error = error_;
  r.zeroed = zeroed_;
  r.position = position_;
  r.pot_position = filtered_position_;
  return r;
}

HallEffectAndPositionZeroingEstimator::HallEffectAndPositionZeroingEstimator(
    const ZeroingConstants &constants)
    : constants_(constants) {
  Reset();
}

void HallEffectAndPositionZeroingEstimator::Reset() {
  offset_ = 0.0;
  min_low_position_ = ::std::numeric_limits<double>::max();
  max_low_position_ = ::std::numeric_limits<double>::lowest();
  zeroed_ = false;
  initialized_ = false;
  last_used_posedge_count_ = 0;
  cycles_high_ = 0;
  high_long_enough_ = false;
  first_start_pos_ = 0.0;
  error_ = false;
  current_ = 0.0;
  first_start_pos_ = 0.0;
}

void HallEffectAndPositionZeroingEstimator::TriggerError() {
  if (!error_) {
    LOG(ERROR, "Manually triggered zeroing error.\n");
    error_ = true;
  }
}

void HallEffectAndPositionZeroingEstimator::StoreEncoderMaxAndMin(
    const HallEffectAndPosition &info) {
  // If we have a new posedge.
  if (!info.current) {
    if (last_hall_) {
      min_low_position_ = max_low_position_ = info.position;
    } else {
      min_low_position_ = ::std::min(min_low_position_, info.position);
      max_low_position_ = ::std::max(max_low_position_, info.position);
    }
  }
  last_hall_ = info.current;
}

void HallEffectAndPositionZeroingEstimator::UpdateEstimate(
    const HallEffectAndPosition &info) {
  // We want to make sure that we encounter at least one posedge while zeroing.
  // So we take the posedge count from the first sample after reset and wait for
  // that count to change and for the hall effect to stay high before we
  // consider ourselves zeroed.
  if (!initialized_) {
    last_used_posedge_count_ = info.posedge_count;
    initialized_ = true;
    last_hall_ = info.current;
  }

  StoreEncoderMaxAndMin(info);

  if (info.current) {
    cycles_high_++;
  } else {
    cycles_high_ = 0;
    last_used_posedge_count_ = info.posedge_count;
  }

  high_long_enough_ = cycles_high_ >= constants_.hall_trigger_zeroing_length;

  bool moving_backward = false;
  if (constants_.zeroing_move_direction) {
    moving_backward = info.position > min_low_position_;
  } else {
    moving_backward = info.position < max_low_position_;
  }

  // If there are no posedges to use or we don't have enough samples yet to
  // have a well-filtered starting position then we use the filtered value as
  // our best guess.
  if (last_used_posedge_count_ != info.posedge_count && high_long_enough_ &&
      moving_backward) {
    // Note the offset and the current posedge count so that we only run this
    // logic once per posedge. That should be more resilient to corrupted
    // intermediate data.
    offset_ = -info.posedge_value;
    if (constants_.zeroing_move_direction) {
      offset_ += constants_.lower_hall_position;
    } else {
      offset_ += constants_.upper_hall_position;
    }
    last_used_posedge_count_ = info.posedge_count;

    // Save the first starting position.
    if (!zeroed_) {
      first_start_pos_ = offset_;
      LOG(INFO, "latching start position %f\n", first_start_pos_);
    }

    // Now that we have an accurate starting position we can consider ourselves
    // zeroed.
    zeroed_ = true;
  }

  position_ = info.position - offset_;
}

HallEffectAndPositionZeroingEstimator::State
HallEffectAndPositionZeroingEstimator::GetEstimatorState() const {
  State r;
  r.error = error_;
  r.zeroed = zeroed_;
  r.encoder = position_;
  r.high_long_enough = high_long_enough_;
  r.offset = offset_;
  return r;
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
  buffered_samples_.clear();
  error_ = false;
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
//
// To guard against the robot moving while updating estimates, buffer a number
// of samples and check that the buffered samples are not different than the
// zeroing threshold. At any point that the samples differ too much, do not
// update estimates based on those samples.
void PotAndAbsEncoderZeroingEstimator::UpdateEstimate(
    const PotAndAbsolutePosition &info) {
  // Check for Abs Encoder NaN value that would mess up the rest of the zeroing
  // code below. NaN values are given when the Absolute Encoder is disconnected.
  if (::std::isnan(info.absolute_encoder)) {
    error_ = true;
    return;
  }

  bool moving = true;
  if (buffered_samples_.size() < constants_.moving_buffer_size) {
    // Not enough samples to start determining if the robot is moving or not,
    // don't use the samples yet.
    buffered_samples_.push_back(info);
  } else {
    // Have enough samples to start determining if the robot is moving or not.
    buffered_samples_[buffered_samples_idx_] = info;
    auto max_value =
        ::std::max_element(buffered_samples_.begin(), buffered_samples_.end(),
                           compare_encoder)
            ->encoder;
    auto min_value =
        ::std::min_element(buffered_samples_.begin(), buffered_samples_.end(),
                           compare_encoder)
            ->encoder;
    if (::std::abs(max_value - min_value) < constants_.zeroing_threshold) {
      // Robot isn't moving, use middle sample to determine offsets.
      moving = false;
    }
  }
  buffered_samples_idx_ =
      (buffered_samples_idx_ + 1) % constants_.moving_buffer_size;

  if (!moving) {
    // The robot is not moving, use the middle sample to determine offsets.
    const int middle_index =
        (buffered_samples_idx_ + (constants_.moving_buffer_size - 1) / 2) %
        constants_.moving_buffer_size;

    // Compute the sum of all the offset samples.
    double relative_to_absolute_offset_sum = 0.0;
    for (size_t i = 0; i < relative_to_absolute_offset_samples_.size(); ++i) {
      relative_to_absolute_offset_sum +=
          relative_to_absolute_offset_samples_[i];
    }

    // Compute the average offset between the absolute encoder and relative
    // encoder.  If we have 0 samples, assume it is 0.
    double average_relative_to_absolute_offset =
        relative_to_absolute_offset_samples_.size() == 0
            ? 0.0
            : relative_to_absolute_offset_sum /
                  relative_to_absolute_offset_samples_.size();

    const double adjusted_incremental_encoder =
        buffered_samples_[middle_index].encoder +
        average_relative_to_absolute_offset;

    // Now, compute the nearest absolute encoder value to the offset relative
    // encoder position.
    const double adjusted_absolute_encoder =
        Wrap(adjusted_incremental_encoder,
             buffered_samples_[middle_index].absolute_encoder -
                 constants_.measured_absolute_position,
             constants_.one_revolution_distance);

    // Reverse the math on the previous line to compute the absolute encoder.
    // Do this by taking the adjusted encoder, and then subtracting off the
    // second argument above, and the value that was added by Wrap.
    filtered_absolute_encoder_ =
        ((buffered_samples_[middle_index].encoder +
          average_relative_to_absolute_offset) -
         (-constants_.measured_absolute_position +
          (adjusted_absolute_encoder -
           (buffered_samples_[middle_index].absolute_encoder -
            constants_.measured_absolute_position))));

    const double relative_to_absolute_offset =
        adjusted_absolute_encoder - buffered_samples_[middle_index].encoder;

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

      relative_to_absolute_offset_samples_.push_back(
          relative_to_absolute_offset);
    } else {
      average_relative_to_absolute_offset -=
          relative_to_absolute_offset_samples_[samples_idx_] /
          relative_to_absolute_offset_samples_size;
      relative_to_absolute_offset_samples_[samples_idx_] =
          relative_to_absolute_offset;
      average_relative_to_absolute_offset +=
          relative_to_absolute_offset /
          relative_to_absolute_offset_samples_size;
    }

    // Now compute the offset between the pot and relative encoder.
    if (offset_samples_.size() < constants_.average_filter_size) {
      offset_samples_.push_back(buffered_samples_[middle_index].pot -
                                buffered_samples_[middle_index].encoder);
    } else {
      offset_samples_[samples_idx_] = buffered_samples_[middle_index].pot -
                                      buffered_samples_[middle_index].encoder;
    }

    // Drop the oldest sample when we run this function the next time around.
    samples_idx_ = (samples_idx_ + 1) % constants_.average_filter_size;

    double pot_relative_encoder_offset_sum = 0.0;
    for (size_t i = 0; i < offset_samples_.size(); ++i) {
      pot_relative_encoder_offset_sum += offset_samples_[i];
    }
    pot_relative_encoder_offset_ =
        pot_relative_encoder_offset_sum / offset_samples_.size();

    offset_ = Wrap(buffered_samples_[middle_index].encoder +
                       pot_relative_encoder_offset_,
                   average_relative_to_absolute_offset +
                       buffered_samples_[middle_index].encoder,
                   constants_.one_revolution_distance) -
              buffered_samples_[middle_index].encoder;
    if (offset_ready()) {
      if (!zeroed_) {
        first_offset_ = offset_;
      }

      if (::std::abs(first_offset_ - offset_) >
          constants_.allowable_encoder_error *
              constants_.one_revolution_distance) {
        LOG(ERROR,
            "Offset moved too far. Initial: %f, current %f, allowable change: "
            "%f\n",
            first_offset_, offset_, constants_.allowable_encoder_error *
                                        constants_.one_revolution_distance);
        error_ = true;
      }

      zeroed_ = true;
    }
  }

  // Update the position.
  filtered_position_ = pot_relative_encoder_offset_ + info.encoder;
  position_ = offset_ + info.encoder;
}

PotAndAbsEncoderZeroingEstimator::State
PotAndAbsEncoderZeroingEstimator::GetEstimatorState() const {
  State r;
  r.error = error_;
  r.zeroed = zeroed_;
  r.position = position_;
  r.pot_position = filtered_position_;
  r.absolute_position = filtered_absolute_encoder_;
  return r;
}

void PulseIndexZeroingEstimator::Reset() {
  max_index_position_ = ::std::numeric_limits<double>::lowest();
  min_index_position_ = ::std::numeric_limits<double>::max();
  offset_ = 0;
  last_used_index_pulse_count_ = 0;
  zeroed_ = false;
  error_ = false;
}

void PulseIndexZeroingEstimator::StoreIndexPulseMaxAndMin(
    const IndexPosition &info) {
  // If we have a new index pulse.
  if (last_used_index_pulse_count_ != info.index_pulses) {
    // If the latest pulses's position is outside the range we've currently
    // seen, record it appropriately.
    if (info.latched_encoder > max_index_position_) {
      max_index_position_ = info.latched_encoder;
    }
    if (info.latched_encoder < min_index_position_) {
      min_index_position_ = info.latched_encoder;
    }
    last_used_index_pulse_count_ = info.index_pulses;
  }
}

int PulseIndexZeroingEstimator::IndexPulseCount() const {
  if (min_index_position_ > max_index_position_) {
    // This condition means we haven't seen a pulse yet.
    return 0;
  }

  // Calculate the number of pulses encountered so far.
  return 1 + static_cast<int>(
                 ::std::round((max_index_position_ - min_index_position_) /
                              constants_.index_difference));
}

void PulseIndexZeroingEstimator::UpdateEstimate(const IndexPosition &info) {
  StoreIndexPulseMaxAndMin(info);
  const int index_pulse_count = IndexPulseCount();
  if (index_pulse_count > constants_.index_pulse_count) {
    error_ = true;
  }

  // TODO(austin): Detect if the encoder or index pulse is unplugged.
  // TODO(austin): Detect missing counts.

  if (index_pulse_count == constants_.index_pulse_count && !zeroed_) {
    offset_ = constants_.measured_index_position -
              constants_.known_index_pulse * constants_.index_difference -
              min_index_position_;
    zeroed_ = true;
  }

  position_ = info.encoder + offset_;
}

PulseIndexZeroingEstimator::State
PulseIndexZeroingEstimator::GetEstimatorState() const {
  State r;
  r.error = error_;
  r.zeroed = zeroed_;
  r.position = position_;
  r.min_index_position = min_index_position_;
  r.max_index_position = max_index_position_;
  r.index_pulses_seen = IndexPulseCount();
  return r;
}

}  // namespace zeroing
}  // namespace frc971
