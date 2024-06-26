#ifndef Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_COLUMN_H_
#define Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_COLUMN_H_

#include "frc971/constants.h"
#include "frc971/zeroing/hall_effect_and_position.h"
#include "frc971/zeroing/zeroing.h"
#include "y2017/constants.h"
#include "y2017/control_loops/superstructure/superstructure_position_generated.h"
#include "y2017/control_loops/superstructure/superstructure_status_generated.h"

namespace y2017::control_loops::superstructure::column {

class ColumnZeroingEstimator {
 public:
  using ZeroingConstants = ::y2017::constants::Values::Column;
  using SubZeroingConstants = ::frc971::constants::HallEffectZeroingConstants;
  using State = ColumnEstimatorState;
  using SubEstimator = ::frc971::zeroing::HallEffectAndPositionZeroingEstimator;

  ColumnZeroingEstimator(const ZeroingConstants &column_constants);

  void UpdateEstimate(const ColumnPosition &position);

  void Reset();

  void TriggerError();

  bool offset_ready() const { return offset_ready_; }

  bool error() const { return error_ || indexer_.error() || turret_.error(); }

  bool zeroed() const {
    return zeroed_ && indexer_.zeroed() && turret_.zeroed();
  }

  double indexer_offset() const { return indexer_offset_; }
  double turret_offset() const { return turret_offset_; }

  // Returns information about our current state.
  flatbuffers::Offset<State> GetEstimatorState(
      flatbuffers::FlatBufferBuilder *fbb) const;

 private:
  // We are ensuring that two subsystems are zeroed, so here they are!
  SubEstimator indexer_, turret_;
  // The offset in positions between the zero indexer and zero turret.
  double indexer_offset_ = 0.0;
  double turret_offset_ = 0.0;
  // Marker to track whether we're fully zeroed yet or not.
  bool zeroed_ = false;
  // Marker to track whether an error has occurred. This gets reset to false
  // whenever Reset() is called.
  bool error_ = false;

  // True if we have seen both edges the first time, but have not seen the
  // region close enough to zero to be convinced which ambiguous start position
  // we started in.
  bool offset_ready_ = false;

  // The max absolute value of the turret angle that we need to get to to be
  // classified as zeroed.  Otherwise, we may be ambiguous on which wrap we
  // are on.
  const double turret_zeroed_distance_;
};

}  // namespace y2017::control_loops::superstructure::column

#endif  // y2017_CONTROL_LOOPS_SUPERSTRUCTURE_COLUMN_H_
