#ifndef Y2017_CONSTANTS_H_
#define Y2017_CONSTANTS_H_

#include <stdint.h>
#include <math.h>

#include "frc971/constants.h"

namespace y2017 {
namespace constants {

// Has all of our "constants", except the ones that come from other places. The
// ones which change between robots are put together with a workable way to
// retrieve the values for the current robot.

// Everything is in SI units (volts, radians, meters, seconds, etc).
// Some of these values are related to the conversion between raw values
// (encoder counts, voltage, etc) to scaled units (radians, meters, etc).
//
// All ratios are from the encoder shaft to the output units.

struct Values {
  // TODO(constants): Update/check these with what we're using this year.

  struct Intake {
    double pot_offset;
    ::frc971::constants::PotAndIndexPulseZeroingConstants zeroing;
  };

  struct Turret {
    double pot_offset;
    ::frc971::constants::PotAndIndexPulseZeroingConstants zeroing;
  };

  struct Hood {
    double pot_offset;
    ::frc971::constants::PotAndIndexPulseZeroingConstants zeroing;
  };

  static const int kZeroingSampleSize = 200;

  // TODO(Brian): This isn't all the way to the output yet.
  static constexpr double kDrivetrainEncoderRatio = 1.0 ;

  static constexpr double kShooterEncoderRatio = 32.0 / 48.0;

  static constexpr double kIntakeEncoderRatio =
      (16.0 * 0.25) * (20.0 / 40.0) / (2.0 * M_PI) * 0.0254;
  static constexpr double kIntakePotRatio = (16 * 0.25) / (2.0 * M_PI) * 0.0254;
  static constexpr double kIntakeEncoderIndexDifference =
      2.0 * M_PI * kIntakeEncoderRatio;

  static constexpr ::frc971::constants::Range kIntakeRange{
      // Lower hard stop in meters (from inches).
      -0.600 * 0.0254,
      // Upper hard stop in meters (from inches).
      8.655 * 0.0254,
      // Lower soft stop in meters (from inches).
      0.000,
      // Upper soft stop in meters (from inches).
      8.530 * 0.0254};

  static constexpr double kHoodEncoderRatio = 20.0 / 345.0;
  static constexpr double kHoodPotRatio = 20.0 / 345.0;
  static constexpr double kHoodEncoderIndexDifference =
      2.0 * M_PI * kHoodEncoderRatio;

  static constexpr ::frc971::constants::Range kHoodRange{
      // Lower hard stop in radians.
      -1.0 / 345.0 * 2.0 * M_PI,
      // Upper hard stop in radians.
      39.0 / 345.0 * 2.0 * M_PI,
      // Lower soft stop in radians.
      0.0,
      // Upper soft stop in radians.
      (39.0 - 1.0) / 345.0 * 2.0 * M_PI};

  static constexpr double kTurretEncoderRatio = 16.0 / 92.0;
  static constexpr double kTurretPotRatio = 16.0 / 92.0;
  static constexpr double kTurretEncoderIndexDifference =
      2.0 * M_PI * kTurretEncoderRatio;

  static constexpr ::frc971::constants::Range kTurretRange{
      // Lower hard stop in radians.
      -460.0 / 2.0 * M_PI / 180.0,
      // Upper hard stop in radians.
      460.0 / 2.0 * M_PI / 180.0,
      // Lower soft stop in radians.
      -450.0 / 2.0 * M_PI / 180.0,
      // Upper soft stop in radians.
      450.0 / 2.0 * M_PI / 180.0};

  static constexpr double kIndexerEncoderRatio = (18.0 / 36.0) * (12.0 / 84.0);
  static constexpr double kIndexerEncoderIndexDifference =
      2.0 * M_PI * kIndexerEncoderRatio;

  double drivetrain_max_speed;

  Intake intake;

  Turret turret;

  Hood hood;

  double down_error;
  const char *vision_name;
};

// Creates (once) a Values instance for ::aos::network::GetTeamNumber() and
// returns a reference to it.
const Values &GetValues();

// Creates Values instances for each team number it is called with and returns
// them.
const Values &GetValuesForTeam(uint16_t team_number);

}  // namespace constants
}  // namespace y2017

#endif  // Y2017_CONSTANTS_H_
