#include "frc971/zeroing/absolute_and_absolute_encoder.h"
#include "y2020/constants.h"

namespace y2020::control_loops::superstructure::hood {

using AbsoluteAndAbsoluteEncoderZeroingEstimator =
    ::frc971::zeroing::AbsoluteAndAbsoluteEncoderZeroingEstimator;

class HoodEncoderZeroingEstimator
    : public AbsoluteAndAbsoluteEncoderZeroingEstimator {
 public:
  HoodEncoderZeroingEstimator(
      const frc971::constants::AbsoluteAndAbsoluteEncoderZeroingConstants
          &constants);

 private:
  double AdjustedSingleTurnAbsoluteEncoder(
      const AbsoluteAndAbsoluteEncoderZeroingEstimator::PositionStruct &sample)
      const override;

  const constants::Values::HoodGeometry hood_geometry_;
};

}  // namespace y2020::control_loops::superstructure::hood
