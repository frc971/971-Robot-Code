#ifndef Y2016_CONSTANTS_H_
#define Y2016_CONSTANTS_H_

#include <stdint.h>

#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/shifter_hall_effect.h"
#include "frc971/constants.h"

namespace y2016 {
namespace constants {

using ::frc971::constants::ShifterHallEffect;
using ::frc971::constants::PotAndIndexPulseZeroingConstants;

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

  // Ratios for our subsystems.
  static constexpr double kShooterEncoderRatio = 1.0;
  static constexpr double kIntakeEncoderRatio =
      14.0 / 64.0 * 18.0 / 72.0 * 16.0 / 48.0;
  static constexpr double kShoulderEncoderRatio =
      14.0 / 64.0 * 18.0 / 72.0 * 12.0 / 42.0;
  static constexpr double kWristEncoderRatio =
      14.0 / 54.0 * 18.0 / 64.0 * 16.0 / 48.0;

  static constexpr double kIntakePotRatio = 18.0 / 72.0 * 16.0 / 48.0;
  static constexpr double kShoulderPotRatio = 12.0 / 42.0;
  static constexpr double kWristPotRatio = 16.0 / 48.0;

  // Difference in radians between index pulses.
  static constexpr double kIntakeEncoderIndexDifference =
      2.0 * M_PI * kIntakeEncoderRatio;
  static constexpr double kShoulderEncoderIndexDifference =
      2.0 * M_PI * kShoulderEncoderRatio;
  static constexpr double kWristEncoderIndexDifference =
      2.0 * M_PI * kWristEncoderRatio;

  // Subsystem motion ranges, in whatever units that their respective queues say
  // the use.
  static constexpr ::frc971::constants::Range kIntakeRange{// Lower hard stop
                                                           -0.5,
                                                           // Upper hard stop
                                                           2.85 + 0.05,
                                                           // Lower soft stop
                                                           -0.300,
                                                           // Uppper soft stop
                                                           2.725};
  static constexpr ::frc971::constants::Range kShoulderRange{// Lower hard stop
                                                             -0.150,
                                                             // Upper hard stop
                                                             2.8,
                                                             // Lower soft stop
                                                             -0.010,
                                                             // Uppper soft stop
                                                             2.0};
  static constexpr ::frc971::constants::Range kWristRange{// Lower hard stop
                                                          -3.0,
                                                          // Upper hard stop
                                                          3.0,
                                                          // Lower soft stop
                                                          -2.6,
                                                          // Uppper soft stop
                                                          2.6};

  // ///// Dynamic constants. /////
  double drivetrain_max_speed;

  struct Intake {
    double pot_offset;
    PotAndIndexPulseZeroingConstants zeroing;
  };
  Intake intake;

  struct Shoulder {
    double pot_offset;
    PotAndIndexPulseZeroingConstants zeroing;
  };
  Shoulder shoulder;

  struct Wrist {
    double pot_offset;
    PotAndIndexPulseZeroingConstants zeroing;
  };
  Wrist wrist;

  const double down_error;
  const char *vision_name;
};

// Creates (once) a Values instance for ::aos::network::GetTeamNumber() and
// returns a reference to it.
const Values &GetValues();

// Creates Values instances for each team number it is called with and returns
// them.
const Values &GetValuesForTeam(uint16_t team_number);

}  // namespace constants
}  // namespace y2016

#endif  // Y2016_CONSTANTS_H_
