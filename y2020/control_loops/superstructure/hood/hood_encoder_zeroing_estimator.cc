#include <cmath>

#include "y2020/control_loops/superstructure/hood/hood_encoder_zeroing_estimator.h"

namespace y2020::control_loops::superstructure::hood {

HoodEncoderZeroingEstimator::HoodEncoderZeroingEstimator(
    const frc971::constants::AbsoluteAndAbsoluteEncoderZeroingConstants
        &constants)
    : AbsoluteAndAbsoluteEncoderZeroingEstimator(constants),
      hood_geometry_(constants::GetValues().hood_geometry) {
  constants::InitValues();
}

double HoodEncoderZeroingEstimator::AdjustedSingleTurnAbsoluteEncoder(
    const AbsoluteAndAbsoluteEncoderZeroingEstimator::PositionStruct &sample)
    const {
  // Using equation derived in
  // https://slack-files.com/T2752T5SA-F02JJ39RY03-f83f22b78d
  // In that equation, theta = theta, l = screw_length, b = diagonal_length,
  // a = back_plate_diagonal_length, and r = radius.
  const double theta =
      frc971::zeroing::AbsoluteAndAbsoluteEncoderZeroingEstimator::
          AdjustedSingleTurnAbsoluteEncoder(sample) +
      hood_geometry_.theta_0;

  const double screw_length =
      std::sqrt(std::pow(hood_geometry_.radius, 2) +
                std::pow(hood_geometry_.diagonal_length, 2) -
                (2 * hood_geometry_.radius * hood_geometry_.diagonal_length *
                 std::cos(theta)) -
                std::pow(hood_geometry_.back_plate_diagonal_length, 2)) -
      hood_geometry_.screw_length_0;
  constexpr double kMToIn = 39.3701;
  const double adjusted_single_turn_absolute_encoder =
      (screw_length * kMToIn) *
      constants::Values::kHoodEncoderRadiansPerInTravel();

  return adjusted_single_turn_absolute_encoder;
}

}  // namespace y2020::control_loops::superstructure::hood
