#ifndef Y2016_CONSTANTS_H_
#define Y2016_CONSTANTS_H_

#include <stdint.h>

#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/shifter_hall_effect.h"
#include "frc971/constants.h"

#define INTAKE_ENCODER_RATIO (16.0 / 58.0 * 18.0 / 72.0 * 14.0 / 64.0)
#define INTAKE_POT_RATIO (16.0 / 58.0)
#define INTAKE_ENCODER_INDEX_DIFFERENCE (2.0 * M_PI * INTAKE_ENCODER_RATIO)

#define SHOULDER_ENCODER_RATIO (16.0 / 58.0 * 18.0 / 72.0 * 14.0 / 64.0)
#define SHOULDER_POT_RATIO (16.0 / 58.0)
#define SHOULDER_ENCODER_INDEX_DIFFERENCE (2.0 * M_PI * SHOULDER_ENCODER_RATIO)

#define WRIST_ENCODER_RATIO (16.0 / 48.0 * 18.0 / 64.0 * 14.0 / 54.0)
#define WRIST_POT_RATIO (16.0 / 48.0)
#define WRIST_ENCODER_INDEX_DIFFERENCE (2.0 * M_PI * WRIST_ENCODER_RATIO)

namespace y2016 {
namespace constants {

using ::frc971::constants::ShifterHallEffect;
using ::frc971::constants::ZeroingConstants;

// Has all of the numbers that change for both robots and makes it easy to
// retrieve the values for the current one.

// Everything is in SI units (volts, radians, meters, seconds, etc).
// Some of these values are related to the conversion between raw values
// (encoder counts, voltage, etc) to scaled units (radians, meters, etc).

// This structure contains current values for all of the things that change.
struct Values {
  // The ratio from the encoder shaft to the drivetrain wheels.
  double drivetrain_encoder_ratio;

  // The gear ratios from motor shafts to the drivetrain wheels for high and low
  // gear.
  double low_gear_ratio;
  double high_gear_ratio;
  ShifterHallEffect left_drive, right_drive;

  double turn_width;

  ::std::function<StateFeedbackLoop<2, 2, 2>()> make_v_drivetrain_loop;
  ::std::function<StateFeedbackLoop<4, 2, 2>()> make_drivetrain_loop;

  double drivetrain_max_speed;

  // Defines a range of motion for a subsystem.
  // These are all absolute positions in scaled units.
  struct Range {
    double lower_hard;
    double upper_hard;
    double lower;
    double upper;
  };

  struct Intake {
    Range limits;
    ZeroingConstants zeroing;
  };
  Intake intake;

  struct Shoulder {
    Range limits;
    ZeroingConstants zeroing;
  };
  Shoulder shoulder;

  struct Wrist {
    Range limits;
    ZeroingConstants zeroing;
  };
  Wrist wrist;
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
