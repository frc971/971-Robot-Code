#ifndef FRC971_ZEROING_ZEROING_H_
#define FRC971_ZEROING_ZEROING_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "flatbuffers/flatbuffers.h"

#include "frc971/constants.h"
#include "frc971/control_loops/control_loops_generated.h"

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
  virtual ~ZeroingEstimator() {}

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

// A trivial ZeroingEstimator which just passes the position straight through.
class RelativeEncoderZeroingEstimator
    : public ZeroingEstimator<RelativePosition,
                              constants::RelativeEncoderZeroingConstants,
                              RelativeEncoderEstimatorState> {
 public:
  explicit RelativeEncoderZeroingEstimator(
      const constants::RelativeEncoderZeroingConstants &) {}

  // Update position with new position from encoder
  void UpdateEstimate(const RelativePosition &position) override {
    position_ = position.encoder();
  }

  // We alre always zeroed
  bool zeroed() const override { return true; }

  // Starting position of the joint
  double offset() const override { return 0; }

  // Has an error occured? Note: Only triggered by TriggerError()
  bool error() const override { return error_; }

  // Offset is always ready, since we are always zeroed.
  bool offset_ready() const override { return true; }

  void TriggerError() override { error_ = true; }

  void Reset() override { error_ = false; }

  flatbuffers::Offset<State> GetEstimatorState(
      flatbuffers::FlatBufferBuilder *fbb) const override {
    State::Builder builder(*fbb);
    builder.add_error(error_);
    builder.add_position(position_);
    return builder.Finish();
  }

 private:
  // Position from encoder relative to start
  double position_ = 0;
  bool error_ = false;
};

}  // namespace zeroing
}  // namespace frc971

// TODO(Brian): Actually split these targets apart. Need to convert all the
// reverse dependencies to #include what they actually need...
#include "frc971/zeroing/absolute_and_absolute_encoder.h"
#include "frc971/zeroing/absolute_encoder.h"
#include "frc971/zeroing/continuous_absolute_encoder.h"
#include "frc971/zeroing/hall_effect_and_position.h"
#include "frc971/zeroing/pot_and_absolute_encoder.h"
#include "frc971/zeroing/pot_and_index.h"
#include "frc971/zeroing/pulse_index.h"

#endif  // FRC971_ZEROING_ZEROING_H_
