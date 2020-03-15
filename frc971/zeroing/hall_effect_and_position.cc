#include "frc971/zeroing/hall_effect_and_position.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "glog/logging.h"

namespace frc971 {
namespace zeroing {

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

}  // namespace zeroing
}  // namespace frc971
