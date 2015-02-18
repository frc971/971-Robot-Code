#ifndef FRC971_CONSTANTS_H_
#define FRC971_CONSTANTS_H_
#include <stdint.h>

#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/shifter_hall_effect.h"

namespace frc971 {
namespace constants {

// Has all of the numbers that change for both robots and makes it easy to
// retrieve the values for the current one.

// Everything is in SI units (volts, radians, meters, seconds, etc).
// Some of these values are related to the conversion between raw values
// (encoder counts, voltage, etc) to scaled units (radians, meters, etc).

// This structure contains current values for all of the things that change.
struct Values {
  // Drivetrain Values /////

  // The ratio from the encoder shaft to the drivetrain wheels.
  double drivetrain_encoder_ratio;
  // The ratio from the encoder shaft to the arm joint.
  double arm_encoder_ratio;
  // The ratio from the pot shaft to the arm joint.
  double arm_pot_ratio;
  // The ratio from the encoder shaft to the elevator output pulley.
  double elev_encoder_ratio;
  // The ratio from the pot shaft to the elevator output pulley.
  double elev_pot_ratio;
  // How far the elevator moves (meters) per radian on the output pulley.
  double elev_distance_per_radian;
  // The ratio from the encoder shaft to the claw joint.
  double claw_encoder_ratio;
  // The ratio from the pot shaft to the claw joint.
  double claw_pot_ratio;

  // How tall a tote is in meters.
  double tote_height;

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
    // The difference in scaled units between two index pulses.
    double index_difference;
    // The absolute position in scaled units of one of the index pulses.
    double measured_index_position;
  };

  // Defines a range of motion for a subsystem.
  // These are all absolute positions in scaled units.
  struct Range {
    double lower_hard_limit;
    double upper_hard_limit;
    double lower_limit;
    double upper_limit;
  };

  struct Claw {
    Range wrist;
    ZeroingConstants zeroing;
    // The value to add to potentiometer readings after they have been converted
    // to radians so that the resulting value is 0 when the claw is at absolute
    // 0 (horizontal straight out the front).
    double potentiometer_offset;

    // Time between sending commands to claw opening pistons and them reaching
    // the new state.
    double piston_switch_time;
    // How far on either side we look for the index pulse before we give up.
    double zeroing_range;
  };
  Claw claw;

  struct Fridge {
    Range elevator;
    Range arm;

    ZeroingConstants left_elev_zeroing;
    ZeroingConstants right_elev_zeroing;
    ZeroingConstants left_arm_zeroing;
    ZeroingConstants right_arm_zeroing;

    // Values to add to scaled potentiometer readings so 0 lines up with the
    // physical absolute 0.
    double left_elevator_potentiometer_offset;
    double right_elevator_potentiometer_offset;
    double left_arm_potentiometer_offset;
    double right_arm_potentiometer_offset;

    // How high the elevator has to be before we start zeroing the arm.
    double arm_zeroing_height;
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
