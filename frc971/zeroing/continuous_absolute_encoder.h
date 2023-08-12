#ifndef FRC971_ZEROING_CONTINUOUS_ABSOLUTE_ENCODER_H_
#define FRC971_ZEROING_CONTINUOUS_ABSOLUTE_ENCODER_H_

#include <vector>

#include "flatbuffers/flatbuffers.h"

#include "frc971/zeroing/zeroing.h"

namespace frc971 {
namespace zeroing {

// Estimates the position with an absolute encoder which spins continuously. The
// absolute encoder must have a 1:1 ratio to the output.
// The provided offset(), when added to incremental encoder, may return a value
// outside of +/- pi. The user is responsible for handling wrapping.
class ContinuousAbsoluteEncoderZeroingEstimator
    : public ZeroingEstimator<
          AbsolutePosition,
          constants::ContinuousAbsoluteEncoderZeroingConstants,
          AbsoluteEncoderEstimatorState> {
 public:
  explicit ContinuousAbsoluteEncoderZeroingEstimator(
      const constants::ContinuousAbsoluteEncoderZeroingConstants &constants);

  // Resets the internal logic so it needs to be re-zeroed.
  void Reset() override;

  // Updates the sensor values for the zeroing logic.
  void UpdateEstimate(const AbsolutePosition &info) override;

  void TriggerError() override { error_ = true; }

  bool zeroed() const override { return zeroed_; }

  double offset() const override { return offset_; }

  bool error() const override { return error_; }

  // Returns true if the sample buffer is full.
  bool offset_ready() const override {
    return relative_to_absolute_offset_samples_.size() ==
           constants_.average_filter_size;
  }

  // Returns information about our current state.
  virtual flatbuffers::Offset<State> GetEstimatorState(
      flatbuffers::FlatBufferBuilder *fbb) const override;

 private:
  struct PositionStruct {
    PositionStruct(const AbsolutePosition &position_buffer)
        : absolute_encoder(position_buffer.absolute_encoder()),
          encoder(position_buffer.encoder()) {}
    double absolute_encoder;
    double encoder;
  };

  // The zeroing constants used to describe the configuration of the system.
  const constants::ContinuousAbsoluteEncoderZeroingConstants constants_;

  // True if the mechanism is zeroed.
  bool zeroed_;
  // Marker to track whether an error has occurred.
  bool error_;
  // The first valid offset we recorded. This is only set after zeroed_ first
  // changes to true.
  double first_offset_;

  // The filtered absolute encoder.  This is used in the status for calibration.
  double filtered_absolute_encoder_ = 0.0;

  // Samples of the offset needed to line the relative encoder up with the
  // absolute encoder.
  ::std::vector<double> relative_to_absolute_offset_samples_;

  MoveDetector<PositionStruct, AbsolutePosition> move_detector_;

  // Estimated start position of the mechanism
  double offset_ = 0;
  // The next position in 'relative_to_absolute_offset_samples_' and
  // 'encoder_samples_' to be used to store the next sample.
  int samples_idx_ = 0;

  // Number of NANs we've seen in a row.
  size_t nan_samples_ = 0;

  // The filtered position.
  double position_ = 0.0;

  // Marker to track what kind of error has occured.
  aos::ErrorList<ZeroingError> errors_;
};

}  // namespace zeroing
}  // namespace frc971

#endif  // FRC971_ZEROING_CONTINUOUS_ABSOLUTE_ENCODER_H_
