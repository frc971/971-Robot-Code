#ifndef FRC971_CONSTANTS_H_
#define FRC971_CONSTANTS_H_
#include <stdint.h>

#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/shifter_hall_effect.h"

namespace frc971 {
namespace constants {

// Has all of the numbers that change for both robots and makes it easy to
// retrieve the values for the current one.

// This structure contains current values for all of the things that change.
struct Values {
  // Drivetrain Values /////

  // The ratio from the encoder shaft to the drivetrain wheels.
  double drivetrain_encoder_ratio;

  // The gear ratios from motor shafts to the drivetrain wheels for high and low
  // gear.
  double low_gear_ratio;
  double high_gear_ratio;
  ShifterHallEffect left_drive, right_drive;
  bool clutch_transmission;

  double turn_width;

  ::std::function<StateFeedbackLoop<2, 2, 2>()> make_v_drivetrain_loop;
  ::std::function<StateFeedbackLoop<4, 2, 2>()> make_drivetrain_loop;

  double drivetrain_done_distance;
  double drivetrain_max_speed;


  // Superstructure Values /////

  struct ZeroingConstants {
    // The number of samples in the moving average filter.
    int average_filter_size;
    // The difference in SI units between two index pulses.
    double index_difference;
    // The absolute position in SI units of one of the index pulses.
    double measured_index_position;
  };

  ZeroingConstants left_arm_zeroing_constants;
  ZeroingConstants right_arm_zeroing_constants;
  ZeroingConstants left_elevator_zeroing_constants;
  ZeroingConstants right_elevator_zeroing_constants;
  ZeroingConstants claw_zeroing_constants;

  // Defines a range of motion for a subsystem.
  struct Range {
    double lower_hard_limit;
    double upper_hard_limit;
    double lower_limit;
    double upper_limit;
  };

  struct Claw {
    Range wrist;
  };
  Claw claw;

  struct Fridge {
    Range elevator;
    Range arm;
  };
  Fridge fridge;

  double max_allowed_left_right_arm_difference;
  double max_allowed_left_right_elevator_difference;
};

// Creates (once) a Values instance for ::aos::network::GetTeamNumber() and
// returns a reference to it.
const Values &GetValues();

// Creates Values instances for each team number it is called with and returns
// them.
const Values &GetValuesForTeam(uint16_t team_number);

}  // namespace constants
}  // namespace frc971

#endif  // FRC971_CONSTANTS_H_
