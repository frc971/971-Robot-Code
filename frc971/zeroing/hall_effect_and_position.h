#ifndef FRC971_ZEROING_HALL_EFFECT_AND_POSITION_H_
#define FRC971_ZEROING_HALL_EFFECT_AND_POSITION_H_

#include "flatbuffers/flatbuffers.h"

#include "frc971/zeroing/zeroing.h"

namespace frc971 {
namespace zeroing {

// Estimates the position with an incremental encoder and a hall effect sensor.
class HallEffectAndPositionZeroingEstimator
    : public ZeroingEstimator<HallEffectAndPosition,
                              constants::HallEffectZeroingConstants,
                              HallEffectAndPositionEstimatorState> {
 public:
  explicit HallEffectAndPositionZeroingEstimator(
      const ZeroingConstants &constants);

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

}  // namespace zeroing
}  // namespace frc971

#endif  // FRC971_ZEROING_HALL_EFFECT_AND_POSITION_H_
