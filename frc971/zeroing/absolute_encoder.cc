#include "frc971/zeroing/absolute_encoder.h"

#include <cmath>
#include <numeric>

#include "glog/logging.h"

#include "aos/containers/error_list.h"
#include "frc971/zeroing/wrap.h"

namespace frc971 {
namespace zeroing {

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
        errors_.Set(ZeroingError::OFFSET_MOVED_TOO_FAR);
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
