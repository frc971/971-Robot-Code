#ifndef Y2015_CONSTANTS_H_
#define Y2015_CONSTANTS_H_

#include <stdint.h>

#include "aos/common/time.h"
#include "frc971/constants.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/shifter_hall_effect.h"

namespace y2015 {
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
  ::frc971::constants::ShifterHallEffect left_drive, right_drive;
  bool clutch_transmission;

  double turn_width;

  ::std::function<StateFeedbackLoop<2, 2, 2>()> make_v_drivetrain_loop;
  ::std::function<StateFeedbackLoop<4, 2, 2>()> make_drivetrain_loop;

  double drivetrain_max_speed;

  // Superstructure Values /////

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
    ::frc971::constants::PotAndIndexPulseZeroingConstants zeroing;
    // The value to add to potentiometer readings after they have been converted
    // to radians so that the resulting value is 0 when the claw is at absolute
    // 0 (horizontal straight out the front).
    double potentiometer_offset;

    // Time between sending commands to claw opening pistons and them reaching
    // the new state.
    ::aos::monotonic_clock::duration piston_switch_time;
    // How far on either side we look for the index pulse before we give up.
    double zeroing_range;
  };
  Claw claw;

  struct Fridge {
    Range elevator;
    Range arm;

    ::frc971::constants::PotAndIndexPulseZeroingConstants left_elev_zeroing;
    ::frc971::constants::PotAndIndexPulseZeroingConstants right_elev_zeroing;
    ::frc971::constants::PotAndIndexPulseZeroingConstants left_arm_zeroing;
    ::frc971::constants::PotAndIndexPulseZeroingConstants right_arm_zeroing;

    // Values to add to scaled potentiometer readings so 0 lines up with the
    // physical absolute 0.
    double left_elevator_potentiometer_offset;
    double right_elevator_potentiometer_offset;
    double left_arm_potentiometer_offset;
    double right_arm_potentiometer_offset;

    // How high the elevator has to be before we start zeroing the arm.
    double arm_zeroing_height;

    // The length of the arm, from the axis of the bottom pivot to the axis of
    // the top pivot.
    double arm_length;
  };
  Fridge fridge;

  double max_allowed_left_right_arm_difference;
  double max_allowed_left_right_elevator_difference;

  struct ClawGeometry {
    // Horizontal distance from the center of the grabber to the end.
    double grabber_half_length;
    // Vertical distance from the arm rotation center to the bottom of the
    // grabber.  Distance measured with arm vertical (theta = 0).
    double grabber_delta_y;
    // Vertical separation of the claw and arm rotation centers with the
    // elevator at 0.0 and the arm angle set to zero.
    double grabber_arm_vert_separation;
    // Horizontal separation of the claw and arm rotation centers with the
    // elevator at 0.0 and the arm angle set to zero.
    double grabber_arm_horz_separation;
    // Distance between the center of the claw to the top of the claw.
    // The line drawn at this distance parallel to the claw centerline is used
    // to determine if claw interfears with the grabber.
    double claw_top_thickness;
    // The grabber is safe at any height if it is behind this location.
    double grabber_always_safe_h_min;
    // The grabber is safe at any x if it is above this location.
    double grabber_always_safe_x_max;
  };
  ClawGeometry clawGeometry;
};

// Creates (once) a Values instance for ::aos::network::GetTeamNumber() and
// returns a reference to it.
const Values &GetValues();

// Creates Values instances for each team number it is called with and returns
// them.
const Values &GetValuesForTeam(uint16_t team_number);

}  // namespace constants
}  // namespace y2015

#endif  // Y2015_CONSTANTS_H_
