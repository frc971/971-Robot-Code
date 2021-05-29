#ifndef frc971_zeroing_absolute_and_absolute_encoder_h_
#define frc971_zeroing_absolute_and_absolute_encoder_h_

#include <vector>

#include "flatbuffers/flatbuffers.h"

#include "aos/containers/error_list.h"
#include "frc971/zeroing/zeroing.h"

namespace frc971 {
namespace zeroing {

// Estimates the position with an absolute encoder which also reports
// incremental counts and an absolute encoder that's not allowed to turn more
// than once. This is like the PotAndAbsoluteEncoderZeroingEstimator, but it
// uses the single turn absolute encoder instead of the potentiometer.
class AbsoluteAndAbsoluteEncoderZeroingEstimator
    : public ZeroingEstimator<
          AbsoluteAndAbsolutePosition,
          constants::AbsoluteAndAbsoluteEncoderZeroingConstants,
          AbsoluteAndAbsoluteEncoderEstimatorState> {
 public:
  explicit AbsoluteAndAbsoluteEncoderZeroingEstimator(
      const constants::AbsoluteAndAbsoluteEncoderZeroingConstants &constants);

  // Resets the internal logic so it needs to be re-zeroed.
  void Reset() override;

  // Updates the sensor values for the zeroing logic.
  void UpdateEstimate(const AbsoluteAndAbsolutePosition &info) override;

  void TriggerError() override { error_ = true; }

  bool zeroed() const override { return zeroed_; }

  double offset() const override { return first_offset_; }

  bool error() const override { return error_; }

  // Returns true if the sample buffer is full.
  bool offset_ready() const override {
    return relative_to_absolute_offset_samples_.size() ==
               constants_.average_filter_size &&
           offset_samples_.size() == constants_.average_filter_size;
  }

  // Returns information about our current state.
  virtual flatbuffers::Offset<State> GetEstimatorState(
      flatbuffers::FlatBufferBuilder *fbb) const override;

 protected:
  struct PositionStruct {
    PositionStruct(const AbsoluteAndAbsolutePosition &position_buffer)
        : single_turn_absolute_encoder(
              position_buffer.single_turn_absolute_encoder()),
          absolute_encoder(position_buffer.absolute_encoder()),
          encoder(position_buffer.encoder()) {}
    double single_turn_absolute_encoder;
    double absolute_encoder;
    double encoder;
  };

  // Returns an adjusted single turn absolute encoder reading.
  // Filled in by default but can be overriden.
  virtual double AdjustedSingleTurnAbsoluteEncoder(
      const PositionStruct &sample) const;

 private:
  // The zeroing constants used to describe the configuration of the system.
  const constants::AbsoluteAndAbsoluteEncoderZeroingConstants constants_;

  // True if the mechanism is zeroed.
  bool zeroed_;
  // Marker to track whether an error has occurred.
  bool error_;
  // Marker to track what kind of error has occured.
  aos::ErrorList<ZeroingError> errors_;
  // The first valid offset we recorded. This is only set after zeroed_ first
  // changes to true.
  double first_offset_;

  // The filtered absolute encoder.  This is used in the status for calibration.
  double filtered_absolute_encoder_ = 0.0;

  // The filtered single turn absolute encoder. This is used in the status for
  // calibration.
  double filtered_single_turn_absolute_encoder_ = 0.0;

  // Samples of the offset needed to line the relative encoder up with the
  // absolute encoder.
  ::std::vector<double> relative_to_absolute_offset_samples_;

  // Offset between the single turn absolute encoder and relative encoder
  // position.
  ::std::vector<double> offset_samples_;

  MoveDetector<PositionStruct, AbsoluteAndAbsolutePosition> move_detector_;

  // Estimated offset between the single turn encoder and relative encoder.
  double single_turn_to_relative_encoder_offset_ = 0;
  // Estimated start position of the mechanism
  double offset_ = 0;
  // The next position in 'relative_to_absolute_offset_samples_' and
  // 'encoder_samples_' to be used to store the next sample.
  int samples_idx_ = 0;

  size_t nan_samples_ = 0;

  // The unzeroed filtered position.
  double filtered_position_ = 0.0;
  // The filtered position.
  double position_ = 0.0;
};

}  // namespace zeroing
}  // namespace frc971

#endif  // FRC971_ZEROING_ABSOLUTE_AND_ABSOLUTE_ENCODER_H_
