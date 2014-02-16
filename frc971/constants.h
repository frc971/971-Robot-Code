#ifndef FRC971_CONSTANTS_H_
#define FRC971_CONSTANTS_H_

#include <stdint.h>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace constants {

// Has all of the numbers that change for both robots and makes it easy to
// retrieve the values for the current one.

const uint16_t kCompTeamNumber = 8971;
const uint16_t kPracticeTeamNumber = 971;

// Contains the voltages for an analog hall effect sensor on a shifter.
struct ShifterHallEffect {
  // The numbers to use for scaling raw voltages to 0-1.
  double high, low;

  // The numbers for when the dog is clear of each gear.
  double clear_high, clear_low;
};

// This structure contains current values for all of the things that change.
struct Values {
  // The ratio from the encoder shaft to the drivetrain wheels.
  double drivetrain_encoder_ratio;

  // The gear ratios from motor shafts to the drivetrain wheels for high and low
  // gear.
  double low_gear_ratio;
  double high_gear_ratio;
  
  double shooter_voltage;
  double shooter_lower_physical_limit;
  double shooter_upper_physical_limit;
  double shooter_hall_effect_start_position;
  double shooter_zeroing_off_speed;
  double shooter_zeroing_speed;
  double position;

  ShifterHallEffect left_drive, right_drive;

  bool clutch_transmission;

  ::std::function<StateFeedbackLoop<2, 2, 2>()> make_v_drivetrain_loop;
  ::std::function<StateFeedbackLoop<4, 2, 2>()> make_drivetrain_loop;
};

// Creates (once) a Values instance and returns a reference to it.
const Values &GetValues();

}  // namespace constants
}  // namespace frc971

#endif  // FRC971_CONSTANTS_H_
