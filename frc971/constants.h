#ifndef FRC971_CONSTANTS_H_
#define FRC971_CONSTANTS_H_

#include <cstddef>

#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/zeroing/constants_generated.h"

namespace frc971 {
namespace constants {

typedef frc971::zeroing::HallEffectZeroingConstantsT HallEffectZeroingConstants;

typedef frc971::zeroing::PotAndIndexPulseZeroingConstantsT
    PotAndIndexPulseZeroingConstants;

typedef frc971::zeroing::EncoderPlusIndexZeroingConstantsT
    EncoderPlusIndexZeroingConstants;

typedef frc971::zeroing::PotAndAbsoluteEncoderZeroingConstantsT
    PotAndAbsoluteEncoderZeroingConstants;

typedef frc971::zeroing::RelativeEncoderZeroingConstantsT
    RelativeEncoderZeroingConstants;

typedef frc971::zeroing::ContinuousAbsoluteEncoderZeroingConstantsT
    ContinuousAbsoluteEncoderZeroingConstants;

typedef frc971::zeroing::AbsoluteEncoderZeroingConstantsT
    AbsoluteEncoderZeroingConstants;

typedef frc971::zeroing::AbsoluteAndAbsoluteEncoderZeroingConstantsT
    AbsoluteAndAbsoluteEncoderZeroingConstants;

// Defines a range of motion for a subsystem.
// These are all absolute positions in scaled units.
struct Range {
  double lower_hard;
  double upper_hard;
  double lower;
  double upper;

  constexpr double middle() const { return (lower_hard + upper_hard) / 2.0; }
  constexpr double middle_soft() const { return (lower + upper) / 2.0; }

  constexpr double range() const { return upper_hard - lower_hard; }

  static Range FromFlatbuffer(const frc971::Range *range) {
    return {.lower_hard = range->lower_hard(),
            .upper_hard = range->upper_hard(),
            .lower = range->lower(),
            .upper = range->upper()};
  }
};

}  // namespace constants
}  // namespace frc971

#endif  // FRC971_CONSTANTS_H_
