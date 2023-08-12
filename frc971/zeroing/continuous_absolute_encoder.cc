#include "frc971/zeroing/continuous_absolute_encoder.h"

#include <cmath>
#include <numeric>

#include "glog/logging.h"

#include "aos/containers/error_list.h"
#include "frc971/zeroing/wrap.h"

namespace frc971 {
namespace zeroing {

ContinuousAbsoluteEncoderZeroingEstimator::
    ContinuousAbsoluteEncoderZeroingEstimator(
        const constants::ContinuousAbsoluteEncoderZeroingConstants &constants)
    : constants_(constants), move_detector_(constants_.moving_buffer_size) {
  relative_to_absolute_offset_samples_.reserve(constants_.average_filter_size);
  Reset();
}

void ContinuousAbsoluteEncoderZeroingEstimator::Reset() {
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
void ContinuousAbsoluteEncoderZeroingEstimator::UpdateEstimate(
    const AbsolutePosition &info) {
  // Check for Abs Encoder NaN value that would mess up the rest of the zeroing
  // code below. NaN values are given when the Absolute Encoder is disconnected.
  if (::std::isnan(info.absolute_encoder())) {
    if (zeroed_) {
      VLOG(1) << "NAN on absolute encoder.";
      errors_.Set(ZeroingError::LOST_ABSOLUTE_ENCODER);
      error_ = true;
    } else {
      ++nan_samples_;
      VLOG(1) << "NAN on absolute encoder while zeroing " << nan_samples_;
      if (nan_samples_ >= constants_.average_filter_size) {
        errors_.Set(ZeroingError::LOST_ABSOLUTE_ENCODER);
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

    // adjusted_* numbers are nominally in the desired output frame.
    const double adjusted_absolute_encoder =
        sample.absolute_encoder - constants_.measured_absolute_position;

    // Note: If are are near the breakpoint of the absolute encoder, this number
    // will be jitter between numbers that are ~one_revolution_distance apart.
    // For that reason, we rewrap it so that we are not near that boundary.
    const double relative_to_absolute_offset =
        adjusted_absolute_encoder - sample.encoder;

    // To avoid the aforementioned jitter, choose a base value to use for
    // wrapping. When we have no prior samples, just use the current offset.
    // Otherwise, we use an arbitrary prior offset (the stored offsets will all
    // already be wrapped).
    const double relative_to_absolute_offset_wrap_base =
        relative_to_absolute_offset_samples_.size() == 0
            ? relative_to_absolute_offset
            : relative_to_absolute_offset_samples_[0];

    const double relative_to_absolute_offset_wrapped =
        UnWrap(relative_to_absolute_offset_wrap_base,
               relative_to_absolute_offset, constants_.one_revolution_distance);

    const size_t relative_to_absolute_offset_samples_size =
        relative_to_absolute_offset_samples_.size();
    if (relative_to_absolute_offset_samples_size <
        constants_.average_filter_size) {
      relative_to_absolute_offset_samples_.push_back(
          relative_to_absolute_offset_wrapped);
    } else {
      relative_to_absolute_offset_samples_[samples_idx_] =
          relative_to_absolute_offset_wrapped;
    }
    samples_idx_ = (samples_idx_ + 1) % constants_.average_filter_size;

    // Compute the average offset between the absolute encoder and relative
    // encoder. Because we just pushed a value, the size() will never be zero.
    offset_ =
        ::std::accumulate(relative_to_absolute_offset_samples_.begin(),
                          relative_to_absolute_offset_samples_.end(), 0.0) /
        relative_to_absolute_offset_samples_.size();

    // To provide a value that can be used to estimate the
    // measured_absolute_position when zeroing, we just need to output the
    // current absolute encoder value. We could make use of the averaging
    // implicit in offset_ to reduce the noise on this slightly.
    filtered_absolute_encoder_ = sample.absolute_encoder;

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
        errors_.Set(ZeroingError::OFFSET_MOVED_TOO_FAR);
        error_ = true;
      }

      zeroed_ = true;
    }
  }

  // Update the position. Wrap it to reflect the fact that we do not have
  // sufficient information to disambiguate which revolution we are on (also,
  // since this value is primarily meant for debugging, this makes it easier to
  // see that the device is actually at zero without having to divide by 2 *
  // pi).
  position_ =
      Wrap(0.0, offset_ + info.encoder(), constants_.one_revolution_distance);
}

flatbuffers::Offset<ContinuousAbsoluteEncoderZeroingEstimator::State>
ContinuousAbsoluteEncoderZeroingEstimator::GetEstimatorState(
    flatbuffers::FlatBufferBuilder *fbb) const {
  flatbuffers::Offset<flatbuffers::Vector<ZeroingError>> errors_offset =
      errors_.ToFlatbuffer(fbb);

  State::Builder builder(*fbb);
  builder.add_error(error_);
  builder.add_zeroed(zeroed_);
  builder.add_position(position_);
  builder.add_absolute_position(filtered_absolute_encoder_);
  builder.add_errors(errors_offset);
  return builder.Finish();
}

}  // namespace zeroing
}  // namespace frc971
