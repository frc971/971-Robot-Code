#ifndef Y2017_CONSTANTS_H_
#define Y2017_CONSTANTS_H_

#include <stdint.h>

namespace y2017 {
namespace constants {

// Has all of the numbers that change for both robots and makes it easy to
// retrieve the values for the current one.

// Everything is in SI units (volts, radians, meters, seconds, etc).
// Some of these values are related to the conversion between raw values
// (encoder counts, voltage, etc) to scaled units (radians, meters, etc).

// This structure contains current values for all of the things that change.
struct Values {
  // ///// Mutual constants between robots. /////
  // TODO(constants): Update/check these with what we're using this year.
  static const int kZeroingSampleSize = 200;

  // The ratio from the encoder shaft to the drivetrain wheels.
  static constexpr double kDrivetrainEncoderRatio =
      (18.0 / 36.0) /*output reduction*/ * (44.0 / 30.0) /*encoder gears*/;

  // ///// Dynamic constants. /////
  double drivetrain_max_speed;
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
