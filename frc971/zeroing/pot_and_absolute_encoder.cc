#include "frc971/zeroing/pot_and_absolute_encoder.h"

#include <cmath>
#include <numeric>

#include "aos/containers/error_list.h"
#include "frc971/zeroing/wrap.h"
#include "glog/logging.h"

namespace frc971 {
namespace zeroing {

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
        errors_.Set(ZeroingError::OFFSET_MOVED_TOO_FAR);
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
  flatbuffers::Offset<flatbuffers::Vector<ZeroingError>> errors_offset =
      errors_.ToFlatbuffer(fbb);

  State::Builder builder(*fbb);
  builder.add_error(error_);
  builder.add_zeroed(zeroed_);
  builder.add_position(position_);
  builder.add_pot_position(filtered_position_);
  builder.add_absolute_position(filtered_absolute_encoder_);
  builder.add_errors(errors_offset);
  return builder.Finish();
}

}  // namespace zeroing
}  // namespace frc971
