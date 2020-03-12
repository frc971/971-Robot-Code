#include "frc971/zeroing/pulse_index.h"

#include <cmath>
#include <limits>

#include "glog/logging.h"

namespace frc971 {
namespace zeroing {

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

}  // namespace zeroing
}  // namespace frc971
