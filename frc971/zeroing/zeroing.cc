#include "frc971/zeroing/zeroing.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

#include "frc971/zeroing/wrap.h"

#include "flatbuffers/flatbuffers.h"
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
    VLOG(1) << "Manually triggered zeroing error.\n";
    error_ = true;
  }
}

void HallEffectAndPositionZeroingEstimator::StoreEncoderMaxAndMin(
    const HallEffectAndPosition &info) {
  // If we have a new posedge.
  if (!info.current()) {
    if (last_hall_) {
      min_low_position_ = max_low_position_ = info.encoder();
    } else {
      min_low_position_ = ::std::min(min_low_position_, info.encoder());
      max_low_position_ = ::std::max(max_low_position_, info.encoder());
    }
  }
  last_hall_ = info.current();
}

void HallEffectAndPositionZeroingEstimator::UpdateEstimate(
    const HallEffectAndPosition &info) {
  // We want to make sure that we encounter at least one posedge while zeroing.
  // So we take the posedge count from the first sample after reset and wait for
  // that count to change and for the hall effect to stay high before we
  // consider ourselves zeroed.
  if (!initialized_) {
    last_used_posedge_count_ = info.posedge_count();
    initialized_ = true;
    last_hall_ = info.current();
  }

  StoreEncoderMaxAndMin(info);

  if (info.current()) {
    cycles_high_++;
  } else {
    cycles_high_ = 0;
    last_used_posedge_count_ = info.posedge_count();
  }

  high_long_enough_ = cycles_high_ >= constants_.hall_trigger_zeroing_length;

  bool moving_backward = false;
  if (constants_.zeroing_move_direction) {
    moving_backward = info.encoder() > min_low_position_;
  } else {
    moving_backward = info.encoder() < max_low_position_;
  }

  // If there are no posedges to use or we don't have enough samples yet to
  // have a well-filtered starting position then we use the filtered value as
  // our best guess.
  if (last_used_posedge_count_ != info.posedge_count() && high_long_enough_ &&
      moving_backward) {
    // Note the offset and the current posedge count so that we only run this
    // logic once per posedge. That should be more resilient to corrupted
    // intermediate data.
    offset_ = -info.posedge_value();
    if (constants_.zeroing_move_direction) {
      offset_ += constants_.lower_hall_position;
    } else {
      offset_ += constants_.upper_hall_position;
    }
    last_used_posedge_count_ = info.posedge_count();

    // Save the first starting position.
    if (!zeroed_) {
      first_start_pos_ = offset_;
      VLOG(2) << "latching start position" << first_start_pos_;
    }

    // Now that we have an accurate starting position we can consider ourselves
    // zeroed.
    zeroed_ = true;
  }

  position_ = info.encoder() - offset_;
}

flatbuffers::Offset<HallEffectAndPositionZeroingEstimator::State>
HallEffectAndPositionZeroingEstimator::GetEstimatorState(
    flatbuffers::FlatBufferBuilder *fbb) const {
  State::Builder builder(*fbb);
  builder.add_error(error_);
  builder.add_zeroed(zeroed_);
  builder.add_encoder(position_);
  builder.add_high_long_enough(high_long_enough_);
  builder.add_offset(offset_);
  return builder.Finish();
}

PotAndAbsoluteEncoderZeroingEstimator::PotAndAbsoluteEncoderZeroingEstimator(
    const constants::PotAndAbsoluteEncoderZeroingConstants &constants)
    : constants_(constants), move_detector_(constants_.moving_buffer_size) {
  relative_to_absolute_offset_samples_.reserve(constants_.average_filter_size);
  offset_samples_.reserve(constants_.average_filter_size);
  Reset();
}

void PotAndAbsoluteEncoderZeroingEstimator::Reset() {
  first_offset_ = 0.0;
  pot_relative_encoder_offset_ = 0.0;
  offset_ = 0.0;
  samples_idx_ = 0;
  filtered_position_ = 0.0;
  position_ = 0.0;
  zeroed_ = false;
  nan_samples_ = 0;
  relative_to_absolute_offset_samples_.clear();
  offset_samples_.clear();
  move_detector_.Reset();
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
void PotAndAbsoluteEncoderZeroingEstimator::UpdateEstimate(
    const PotAndAbsolutePosition &info) {
  // Check for Abs Encoder NaN value that would mess up the rest of the zeroing
  // code below. NaN values are given when the Absolute Encoder is disconnected.
  if (::std::isnan(info.absolute_encoder())) {
    if (zeroed_) {
      VLOG(1) << "NAN on absolute encoder.";
      error_ = true;
    } else {
      ++nan_samples_;
      VLOG(1) << "NAN on absolute encoder while zeroing" << nan_samples_;
      if (nan_samples_ >= constants_.average_filter_size) {
        error_ = true;
        zeroed_ = true;
      }
    }
    // Throw some dummy values in for now.
    filtered_absolute_encoder_ = info.absolute_encoder();
    filtered_position_ = pot_relative_encoder_offset_ + info.encoder();
    position_ = offset_ + info.encoder();
    return;
  }

  const bool moving = move_detector_.Update(info, constants_.moving_buffer_size,
                                            constants_.zeroing_threshold);

  if (!moving) {
    const PositionStruct &sample = move_detector_.GetSample();

    // Compute the average offset between the absolute encoder and relative
    // encoder.  If we have 0 samples, assume it is 0.
    double average_relative_to_absolute_offset =
        relative_to_absolute_offset_samples_.size() == 0
            ? 0.0
            : ::std::accumulate(relative_to_absolute_offset_samples_.begin(),
                                relative_to_absolute_offset_samples_.end(),
                                0.0) /
                  relative_to_absolute_offset_samples_.size();

    const double adjusted_incremental_encoder =
        sample.encoder + average_relative_to_absolute_offset;

    // Now, compute the nearest absolute encoder value to the offset relative
    // encoder position.
    const double adjusted_absolute_encoder =
        UnWrap(adjusted_incremental_encoder,
               sample.absolute_encoder - constants_.measured_absolute_position,
               constants_.one_revolution_distance);

    // We can now compute the offset now that we have unwrapped the absolute
    // encoder.
    const double relative_to_absolute_offset =
        adjusted_absolute_encoder - sample.encoder;

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
      offset_samples_.push_back(sample.pot - sample.encoder);
    } else {
      offset_samples_[samples_idx_] = sample.pot - sample.encoder;
    }

    // Drop the oldest sample when we run this function the next time around.
    samples_idx_ = (samples_idx_ + 1) % constants_.average_filter_size;

    pot_relative_encoder_offset_ =
        ::std::accumulate(offset_samples_.begin(), offset_samples_.end(), 0.0) /
        offset_samples_.size();

    offset_ = UnWrap(sample.encoder + pot_relative_encoder_offset_,
                     average_relative_to_absolute_offset + sample.encoder,
                     constants_.one_revolution_distance) -
              sample.encoder;

    // Reverse the math for adjusted_absolute_encoder to compute the absolute
    // encoder. Do this by taking the adjusted encoder, and then subtracting off
    // the second argument above, and the value that was added by Wrap.
    filtered_absolute_encoder_ =
        ((sample.encoder + average_relative_to_absolute_offset) -
         (-constants_.measured_absolute_position +
          (adjusted_absolute_encoder -
           (sample.absolute_encoder - constants_.measured_absolute_position))));

    if (offset_ready()) {
      if (!zeroed_) {
        first_offset_ = offset_;
      }

      if (::std::abs(first_offset_ - offset_) >
          constants_.allowable_encoder_error *
              constants_.one_revolution_distance) {
        VLOG(1) << "Offset moved too far. Initial: " << first_offset_
                << ", current " << offset_ << ", allowable change: "
                << constants_.allowable_encoder_error *
                       constants_.one_revolution_distance;
        error_ = true;
      }

      zeroed_ = true;
    }
  }

  // Update the position.
  filtered_position_ = pot_relative_encoder_offset_ + info.encoder();
  position_ = offset_ + info.encoder();
}

flatbuffers::Offset<PotAndAbsoluteEncoderZeroingEstimator::State>
PotAndAbsoluteEncoderZeroingEstimator::GetEstimatorState(
    flatbuffers::FlatBufferBuilder *fbb) const {
  State::Builder builder(*fbb);
  builder.add_error(error_);
  builder.add_zeroed(zeroed_);
  builder.add_position(position_);
  builder.add_pot_position(filtered_position_);
  builder.add_absolute_position(filtered_absolute_encoder_);
  return builder.Finish();
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
  if (last_used_index_pulse_count_ != info.index_pulses()) {
    // If the latest pulses's position is outside the range we've currently
    // seen, record it appropriately.
    if (info.latched_encoder() > max_index_position_) {
      max_index_position_ = info.latched_encoder();
    }
    if (info.latched_encoder() < min_index_position_) {
      min_index_position_ = info.latched_encoder();
    }
    last_used_index_pulse_count_ = info.index_pulses();
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
    if (!error_) {
      VLOG(1) << "Got more index pulses than expected. Got "
              << index_pulse_count << " expected "
              << constants_.index_pulse_count;
      error_ = true;
    }
  }

  // TODO(austin): Detect if the encoder or index pulse is unplugged.
  // TODO(austin): Detect missing counts.

  if (index_pulse_count == constants_.index_pulse_count && !zeroed_) {
    offset_ = constants_.measured_index_position -
              constants_.known_index_pulse * constants_.index_difference -
              min_index_position_;
    zeroed_ = true;
  } else if (zeroed_ && !error_) {
    // Detect whether the index pulse is somewhere other than where we expect
    // it to be. First we compute the position of the most recent index pulse.
    double index_pulse_distance =
        info.latched_encoder() + offset_ - constants_.measured_index_position;
    // Second we compute the position of the index pulse in terms of
    // the index difference. I.e. if this index pulse is two pulses away from
    // the index pulse that we know about then this number should be positive
    // or negative two.
    double relative_distance =
        index_pulse_distance / constants_.index_difference;
    // Now we compute how far away the measured index pulse is from the
    // expected index pulse.
    double error = relative_distance - ::std::round(relative_distance);
    // This lets us check if the index pulse is within an acceptable error
    // margin of where we expected it to be.
    if (::std::abs(error) > constants_.allowable_encoder_error) {
      VLOG(1)
          << "Encoder ticks out of range since last index pulse. known index "
             "pulse: "
          << constants_.measured_index_position << ", expected index pulse: "
          << round(relative_distance) * constants_.index_difference +
                 constants_.measured_index_position
          << ", actual index pulse: " << info.latched_encoder() + offset_
          << ", "
             "allowable error: "
          << constants_.allowable_encoder_error * constants_.index_difference;
      error_ = true;
    }
  }

  position_ = info.encoder() + offset_;
}

flatbuffers::Offset<PulseIndexZeroingEstimator::State>
PulseIndexZeroingEstimator::GetEstimatorState(
    flatbuffers::FlatBufferBuilder *fbb) const {
  State::Builder builder(*fbb);
  builder.add_error(error_);
  builder.add_zeroed(zeroed_);
  builder.add_position(position_);
  builder.add_min_index_position(min_index_position_);
  builder.add_max_index_position(max_index_position_);
  builder.add_index_pulses_seen(IndexPulseCount());
  return builder.Finish();
}

AbsoluteEncoderZeroingEstimator::AbsoluteEncoderZeroingEstimator(
    const constants::AbsoluteEncoderZeroingConstants &constants)
    : constants_(constants), move_detector_(constants_.moving_buffer_size) {
  relative_to_absolute_offset_samples_.reserve(constants_.average_filter_size);
  Reset();
}

void AbsoluteEncoderZeroingEstimator::Reset() {
  zeroed_ = false;
  error_ = false;
  first_offset_ = 0.0;
  offset_ = 0.0;
  samples_idx_ = 0;
  position_ = 0.0;
  nan_samples_ = 0;
  relative_to_absolute_offset_samples_.clear();
  move_detector_.Reset();
}


// The math here is a bit backwards, but I think it'll be less error prone that
// way and more similar to the version with a pot as well.
//
// We start by unwrapping the absolute encoder using the relative encoder.  This
// puts us in a non-wrapping space and lets us average a bit easier.  From
// there, we can compute an offset and wrap ourselves back such that we stay
// close to the middle value.
//
// To guard against the robot moving while updating estimates, buffer a number
// of samples and check that the buffered samples are not different than the
// zeroing threshold. At any point that the samples differ too much, do not
// update estimates based on those samples.
void AbsoluteEncoderZeroingEstimator::UpdateEstimate(
    const AbsolutePosition &info) {
  // Check for Abs Encoder NaN value that would mess up the rest of the zeroing
  // code below. NaN values are given when the Absolute Encoder is disconnected.
  if (::std::isnan(info.absolute_encoder())) {
    if (zeroed_) {
      VLOG(1) << "NAN on absolute encoder.";
      error_ = true;
    } else {
      ++nan_samples_;
      VLOG(1) << "NAN on absolute encoder while zeroing " << nan_samples_;
      if (nan_samples_ >= constants_.average_filter_size) {
        error_ = true;
        zeroed_ = true;
      }
    }
    // Throw some dummy values in for now.
    filtered_absolute_encoder_ = info.absolute_encoder();
    position_ = offset_ + info.encoder();
    return;
  }

  const bool moving = move_detector_.Update(info, constants_.moving_buffer_size,
                                            constants_.zeroing_threshold);

  if (!moving) {
    const PositionStruct &sample = move_detector_.GetSample();

    // Compute the average offset between the absolute encoder and relative
    // encoder.  If we have 0 samples, assume it is 0.
    double average_relative_to_absolute_offset =
        relative_to_absolute_offset_samples_.size() == 0
            ? 0.0
            : ::std::accumulate(relative_to_absolute_offset_samples_.begin(),
                                relative_to_absolute_offset_samples_.end(),
                                0.0) /
                  relative_to_absolute_offset_samples_.size();

    // Now, compute the estimated absolute position using the previously
    // estimated offset and the incremental encoder.
    const double adjusted_incremental_encoder =
        sample.encoder + average_relative_to_absolute_offset;

    // Now, compute the absolute encoder value nearest to the offset relative
    // encoder position.
    const double adjusted_absolute_encoder =
        UnWrap(adjusted_incremental_encoder,
               sample.absolute_encoder - constants_.measured_absolute_position,
               constants_.one_revolution_distance);

    // We can now compute the offset now that we have unwrapped the absolute
    // encoder.
    const double relative_to_absolute_offset =
        adjusted_absolute_encoder - sample.encoder;

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

    // Drop the oldest sample when we run this function the next time around.
    samples_idx_ = (samples_idx_ + 1) % constants_.average_filter_size;

    // And our offset is the offset that gives us the position within +- ord/2
    // of the middle position.
    offset_ = Wrap(constants_.middle_position,
                   average_relative_to_absolute_offset + sample.encoder,
                   constants_.one_revolution_distance) -
              sample.encoder;

    // Reverse the math for adjusted_absolute_encoder to compute the absolute
    // encoder. Do this by taking the adjusted encoder, and then subtracting off
    // the second argument above, and the value that was added by Wrap.
    filtered_absolute_encoder_ =
        ((sample.encoder + average_relative_to_absolute_offset) -
         (-constants_.measured_absolute_position +
          (adjusted_absolute_encoder -
           (sample.absolute_encoder - constants_.measured_absolute_position))));

    if (offset_ready()) {
      if (!zeroed_) {
        first_offset_ = offset_;
      }

      if (::std::abs(first_offset_ - offset_) >
          constants_.allowable_encoder_error *
              constants_.one_revolution_distance) {
        VLOG(1) << "Offset moved too far. Initial: " << first_offset_
                << ", current " << offset_ << ", allowable change: "
                << constants_.allowable_encoder_error *
                       constants_.one_revolution_distance;
        error_ = true;
      }

      zeroed_ = true;
    }
  }

  // Update the position.
  position_ = offset_ + info.encoder();
}

flatbuffers::Offset<AbsoluteEncoderZeroingEstimator::State>
AbsoluteEncoderZeroingEstimator::GetEstimatorState(
    flatbuffers::FlatBufferBuilder *fbb) const {
  State::Builder builder(*fbb);
  builder.add_error(error_);
  builder.add_zeroed(zeroed_);
  builder.add_position(position_);
  builder.add_absolute_position(filtered_absolute_encoder_);
  return builder.Finish();
}

}  // namespace zeroing
}  // namespace frc971
