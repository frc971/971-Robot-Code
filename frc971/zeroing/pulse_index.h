#ifndef FRC971_ZEROING_PULSE_INDEX_H_
#define FRC971_ZEROING_PULSE_INDEX_H_

#include "flatbuffers/flatbuffers.h"

#include "frc971/zeroing/zeroing.h"

namespace frc971 {
namespace zeroing {

// Zeros by seeing all the index pulses in the range of motion of the mechanism
// and using that to figure out which index pulse is which.
class PulseIndexZeroingEstimator
    : public ZeroingEstimator<IndexPosition,
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

}  // namespace zeroing
}  // namespace frc971

#endif  // FRC971_ZEROING_PULSE_INDEX_H_
