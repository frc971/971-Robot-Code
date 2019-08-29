#ifndef FRC971_ZEROING_ZEROING_H_
#define FRC971_ZEROING_ZEROING_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/constants.h"

#include "flatbuffers/flatbuffers.h"

// TODO(pschrader): Flag an error if encoder index pulse is not n revolutions
// away from the last one (i.e. got extra counts from noise, etc..)
//
// TODO(pschrader): Flag error if the pot disagrees too much with the encoder
// after being zeroed.
//
// TODO(pschrader): Watch the offset over long periods of time and flag if it
// gets too far away from the initial value.

namespace frc971 {
namespace zeroing {

template <typename TPosition, typename TZeroingConstants, typename TState>
class ZeroingEstimator {
 public:
  using Position = TPosition;
  using ZeroingConstants = TZeroingConstants;
  using State = TState;
  virtual ~ZeroingEstimator(){}

  // Returns true if the logic considers the corresponding mechanism to be
  // zeroed.
  virtual bool zeroed() const = 0;

  // Returns the estimated position of the corresponding mechanism.
  virtual double offset() const = 0;

  // Returns true if there has been an error.
  virtual bool error() const = 0;

  // Returns true if an offset is ready.
  virtual bool offset_ready() const = 0;

  // Triggers an internal error. This is used for testing the error
  // logic.
  virtual void TriggerError() = 0;

  // Resets the estimator, clearing error and zeroing states.
  virtual void Reset() = 0;

  // Updates the internal logic with new sensor values
  virtual void UpdateEstimate(const Position &) = 0;

  // Returns the state of the estimator
  virtual flatbuffers::Offset<State> GetEstimatorState(
      flatbuffers::FlatBufferBuilder *fbb) const = 0;
};

// Estimates the position with an incremental encoder with an index pulse and a
// potentiometer.
class PotAndIndexPulseZeroingEstimator : public ZeroingEstimator<PotAndIndexPosition,
    constants::PotAndIndexPulseZeroingConstants,
    EstimatorState> {
 public:
  explicit PotAndIndexPulseZeroingEstimator(
      const constants::PotAndIndexPulseZeroingConstants &constants);

  // Update the internal logic with the next sensor values.
  void UpdateEstimate(const PotAndIndexPosition &info) override;

  // Reset the internal logic so it needs to be re-zeroed.
  void Reset() override;

  // Manually trigger an internal error. This is used for testing the error
  // logic.
  void TriggerError() override;

  bool error() const override { return error_; }

  bool zeroed() const override { return zeroed_; }

  double offset() const override { return offset_; }

  // Returns a number between 0 and 1 that represents the percentage of the
  // samples being used in the moving average filter. A value of 0.0 means that
  // no samples are being used. A value of 1.0 means that the filter is using
  // as many samples as it has room for. For example, after a Reset() this
  // value returns 0.0. As more samples get added with UpdateEstimate(...) the
  // return value starts increasing to 1.0.
  double offset_ratio_ready() const {
    return start_pos_samples_.size() /
           static_cast<double>(constants_.average_filter_size);
  }

  // Returns true if the sample buffer is full.
  bool offset_ready() const override {
    return start_pos_samples_.size() == constants_.average_filter_size;
  }

  // Returns information about our current state.
  virtual flatbuffers::Offset<State> GetEstimatorState(
      flatbuffers::FlatBufferBuilder *fbb) const override;

 private:
  // This function calculates the start position given the internal state and
  // the provided `latched_encoder' value.
  double CalculateStartPosition(double start_average,
                                double latched_encoder) const;

  // The zeroing constants used to describe the configuration of the system.
  const constants::PotAndIndexPulseZeroingConstants constants_;

  // The estimated position.
  double position_;
  // The unzeroed filtered position.
  double filtered_position_ = 0.0;
  // The next position in 'start_pos_samples_' to be used to store the next
  // sample.
  int samples_idx_;
  // Last 'max_sample_count_' samples for start positions.
  std::vector<double> start_pos_samples_;
  // The estimated starting position of the mechanism. We also call this the
  // 'offset' in some contexts.
  double offset_;
  // Flag for triggering logic that takes note of the current index pulse count
  // after a reset. See `last_used_index_pulse_count_'.
  bool wait_for_index_pulse_;
  // After a reset we keep track of the index pulse count with this. Only after
  // the index pulse count changes (i.e. increments at least once or wraps
  // around) will we consider the mechanism zeroed. We also use this to store
  // the most recent `PotAndIndexPosition::index_pulses' value when the start
  // position was calculated. It helps us calculate the start position only on
  // index pulses to reject corrupted intermediate data.
  uint32_t last_used_index_pulse_count_;
  // Marker to track whether we're fully zeroed yet or not.
  bool zeroed_;
  // Marker to track whether an error has occurred. This gets reset to false
  // whenever Reset() is called.
  bool error_;
  // Stores the position "start_pos" variable the first time the program
  // is zeroed.
  double first_start_pos_;
};

// Estimates the position with an incremental encoder and a hall effect sensor.
class HallEffectAndPositionZeroingEstimator
    : public ZeroingEstimator<HallEffectAndPosition,
    constants::HallEffectZeroingConstants,
    HallEffectAndPositionEstimatorState> {
 public:
  explicit HallEffectAndPositionZeroingEstimator(const ZeroingConstants &constants);

  // Update the internal logic with the next sensor values.
  void UpdateEstimate(const Position &info) override;

  // Reset the internal logic so it needs to be re-zeroed.
  void Reset() override;

  // Manually trigger an internal error. This is used for testing the error
  // logic.
  void TriggerError() override;

  bool error() const override { return error_; }

  bool zeroed() const override { return zeroed_; }

  double offset() const override { return offset_; }

  bool offset_ready() const override { return zeroed_; }

  // Returns information about our current state.
  virtual flatbuffers::Offset<State> GetEstimatorState(
      flatbuffers::FlatBufferBuilder *fbb) const override;

 private:
  // Sets the minimum and maximum posedge position values.
  void StoreEncoderMaxAndMin(const HallEffectAndPosition &info);

  // The zeroing constants used to describe the configuration of the system.
  const ZeroingConstants constants_;

  // The estimated state of the hall effect.
  double current_ = 0.0;
  // The estimated position.
  double position_ = 0.0;
  // The smallest and largest positions of the last set of encoder positions
  // while the hall effect was low.
  double min_low_position_;
  double max_low_position_;
  // If we've seen the hall effect high for enough times without going low, then
  // we can be sure it isn't a false positive.
  bool high_long_enough_;
  size_t cycles_high_;

  bool last_hall_ = false;

  // The estimated starting position of the mechanism. We also call this the
  // 'offset' in some contexts.
  double offset_;
  // Flag for triggering logic that takes note of the current posedge count
  // after a reset. See `last_used_posedge_count_'.
  bool initialized_;
  // After a reset we keep track of the posedge count with this. Only after the
  // posedge count changes (i.e. increments at least once or wraps around) will
  // we consider the mechanism zeroed. We also use this to store the most recent
  // `HallEffectAndPosition::posedge_count' value when the start position
  // was calculated. It helps us calculate the start position only on posedges
  // to reject corrupted intermediate data.
  int32_t last_used_posedge_count_;
  // Marker to track whether we're fully zeroed yet or not.
  bool zeroed_;
  // Marker to track whether an error has occurred. This gets reset to false
  // whenever Reset() is called.
  bool error_ = false;
  // Stores the position "start_pos" variable the first time the program
  // is zeroed.
  double first_start_pos_;
};

// Class to encapsulate the logic to decide when we are moving and which samples
// are safe to use.
template <typename Position, typename PositionBuffer>
class MoveDetector {
 public:
  MoveDetector(size_t filter_size) {
    buffered_samples_.reserve(filter_size);
    Reset();
  }

  // Clears all the state.
  void Reset() {
    buffered_samples_.clear();
    buffered_samples_idx_ = 0;
  }

  // Updates the detector with a new sample.  Returns true if we are moving.
  // buffer_size is the number of samples in the moving buffer, and
  // zeroing_threshold is the max amount we can move within the period specified
  // by buffer_size.
  bool Update(const PositionBuffer &position_buffer, size_t buffer_size,
              double zeroing_threshold) {
    bool moving = true;
    Position position(position_buffer);
    if (buffered_samples_.size() < buffer_size) {
      // Not enough samples to start determining if the robot is moving or not,
      // don't use the samples yet.
      buffered_samples_.push_back(position);
    } else {
      // Have enough samples to start determining if the robot is moving or not.
      buffered_samples_[buffered_samples_idx_] = position;
      const auto minmax_value = ::std::minmax_element(
          buffered_samples_.begin(), buffered_samples_.end(),
          [](const Position &left, const Position &right) {
            return left.encoder < right.encoder;
          });
      const double min_value = minmax_value.first->encoder;
      const double max_value = minmax_value.second->encoder;

      if (::std::abs(max_value - min_value) < zeroing_threshold) {
        // Robot isn't moving, use middle sample to determine offsets.
        moving = false;
      }
    }
    buffered_samples_idx_ = (buffered_samples_idx_ + 1) % buffer_size;
    return moving;
  }

  // Returns a safe sample if we aren't moving as reported by Update.
  const Position &GetSample() const {
    // The robot is not moving, use the middle sample to determine offsets.
    // The middle sample makes it so that we don't use the samples from the
    // beginning or end of periods when the robot is moving.
    const int middle_index =
        (buffered_samples_idx_ + (buffered_samples_.size() / 2)) %
        buffered_samples_.size();
    return buffered_samples_[middle_index];
  }

 private:
  // The last buffer_size samples.
  ::std::vector<Position> buffered_samples_;
  // The index to place the next sample in.
  size_t buffered_samples_idx_;
};

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
};

// Zeros by seeing all the index pulses in the range of motion of the mechanism
// and using that to figure out which index pulse is which.
class PulseIndexZeroingEstimator : public ZeroingEstimator<IndexPosition,
    constants::EncoderPlusIndexZeroingConstants,
    IndexEstimatorState> {
 public:
  explicit PulseIndexZeroingEstimator(const ZeroingConstants &constants)
      : constants_(constants) {
    Reset();
  }

  // Resets the internal logic so it needs to be re-zeroed.
  void Reset() override;

  bool zeroed() const override { return zeroed_; }

  // It's as ready as it'll ever be...
  bool offset_ready() const override { return true; }

  double offset() const override { return offset_; }

  bool error() const override { return error_; }

  // Updates the internal logic with the next sensor values.
  void UpdateEstimate(const IndexPosition &info) override;

  // Returns information about our current state.
  virtual flatbuffers::Offset<State> GetEstimatorState(
      flatbuffers::FlatBufferBuilder *fbb) const override;

  void TriggerError() override { error_ = true; }

 private:
  // Returns the current real position using the relative encoder offset.
  double CalculateCurrentPosition(const IndexPosition &info);

  // Sets the minimum and maximum index pulse position values.
  void StoreIndexPulseMaxAndMin(const IndexPosition &info);

  // Returns the number of index pulses we should have seen so far.
  int IndexPulseCount() const;

  // Contains the physical constants describing the system.
  const ZeroingConstants constants_;

  // The smallest position of all the index pulses.
  double min_index_position_;
  // The largest position of all the index pulses.
  double max_index_position_;

  // The estimated starting position of the mechanism.
  double offset_;
  // After a reset we keep track of the index pulse count with this. Only after
  // the index pulse count changes (i.e. increments at least once or wraps
  // around) will we consider the mechanism zeroed. We also use this to store
  // the most recent `PotAndIndexPosition::index_pulses' value when the start
  // position was calculated. It helps us calculate the start position only on
  // index pulses to reject corrupted intermediate data.
  uint32_t last_used_index_pulse_count_;

  // True if we are fully zeroed.
  bool zeroed_;
  // Marker to track whether an error has occurred.
  bool error_;

  // The estimated position.
  double position_;
};

// Estimates the position with an absolute encoder which also reports
// incremental counts.  The absolute encoder can't spin more than one
// revolution.
class AbsoluteEncoderZeroingEstimator
    : public ZeroingEstimator<AbsolutePosition,
                              constants::AbsoluteEncoderZeroingConstants,
                              AbsoluteEncoderEstimatorState> {
 public:
  explicit AbsoluteEncoderZeroingEstimator(
      const constants::AbsoluteEncoderZeroingConstants &constants);

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
  const constants::AbsoluteEncoderZeroingConstants constants_;

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
};

}  // namespace zeroing
}  // namespace frc971

#endif  // FRC971_ZEROING_ZEROING_H_
