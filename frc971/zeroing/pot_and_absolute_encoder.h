#ifndef FRC971_ZEROING_POT_AND_ABSOLUTE_ENCODER_H_
#define FRC971_ZEROING_POT_AND_ABSOLUTE_ENCODER_H_

#include <vector>

#include "aos/containers/error_list.h"
#include "flatbuffers/flatbuffers.h"
#include "frc971/zeroing/zeroing.h"

namespace frc971 {
namespace zeroing {

// Estimates the position with an absolute encoder which also reports
// incremental counts, and a potentiometer.
class PotAndAbsoluteEncoderZeroingEstimator
    : public ZeroingEstimator<PotAndAbsolutePosition,
                              constants::PotAndAbsoluteEncoderZeroingConstants,
                              PotAndAbsoluteEncoderEstimatorState> {
 public:
  explicit PotAndAbsoluteEncoderZeroingEstimator(
      const constants::PotAndAbsoluteEncoderZeroingConstants &constants);

  // Resets the internal logic so it needs to be re-zeroed.
  void Reset() override;

  // Updates the sensor values for the zeroing logic.
  void UpdateEstimate(const PotAndAbsolutePosition &info) override;

  void TriggerError() override { error_ = true; }

  bool zeroed() const override { return zeroed_; }

  double offset() const override { return offset_; }

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

 private:
  struct PositionStruct {
    PositionStruct(const PotAndAbsolutePosition &position_buffer)
        : absolute_encoder(position_buffer.absolute_encoder()),
          encoder(position_buffer.encoder()),
          pot(position_buffer.pot()) {}
    double absolute_encoder;
    double encoder;
    double pot;
  };

  // The zeroing constants used to describe the configuration of the system.
  const constants::PotAndAbsoluteEncoderZeroingConstants constants_;

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
  // Offset between the Pot and Relative encoder position.
  ::std::vector<double> offset_samples_;

  MoveDetector<PositionStruct, PotAndAbsolutePosition> move_detector_;

  // Estimated offset between the pot and relative encoder.
  double pot_relative_encoder_offset_ = 0;
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

  // Marker to track what kind of error has occured.
  aos::ErrorList<ZeroingError> errors_;
};

}  // namespace zeroing
}  // namespace frc971

#endif  // FRC971_ZEROING_POT_AND_ABSOLUTE_ENCODER_H_
