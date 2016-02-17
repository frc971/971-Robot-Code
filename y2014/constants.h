#ifndef Y2014_CONSTANTS_H_
#define Y2014_CONSTANTS_H_

#include <stdint.h>

#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/shifter_hall_effect.h"

namespace y2014 {
namespace constants {

using ::frc971::constants::ShifterHallEffect;

// Has all of the numbers that change for both robots and makes it easy to
// retrieve the values for the current one.

// Everything is in SI units (volts, radians, meters, seconds, etc).
// Some of these values are related to the conversion between raw values
// (encoder counts, voltage, etc) to scaled units (radians, meters, etc).

// This structure contains current values for all of the things that change.
struct Values {
  // This is useful for representing the 2 sides of a hall effect sensor etc.
  struct AnglePair {
    // The angles for increasing values (posedge on lower, negedge on upper).
    double lower_angle, upper_angle;
    // The angles for decreasing values (negedge on lower, posedge on upper).
    double lower_decreasing_angle, upper_decreasing_angle;
  };

  // The ratio from the encoder shaft to the drivetrain wheels.
  double drivetrain_encoder_ratio;

  // The gear ratios from motor shafts to the drivetrain wheels for high and low
  // gear.
  double low_gear_ratio;
  double high_gear_ratio;
  ShifterHallEffect left_drive, right_drive;
  bool clutch_transmission;

  ::std::function<StateFeedbackLoop<2, 2, 2>()> make_v_drivetrain_loop;
  ::std::function<StateFeedbackLoop<4, 2, 2>()> make_drivetrain_loop;

  double drivetrain_max_speed;

  struct ZeroingConstants {
    // The number of samples in the moving average filter.
    int average_filter_size;
    // The difference in scaled units between two index pulses.
    double index_difference;
    // The absolute position in scaled units of one of the index pulses.
    double measured_index_position;
    // Value between 0 and 1 which determines a fraction of the index_diff
    // you want to use.
    double allowable_encoder_error;
  };

  // Defines a range of motion for a subsystem.
  // These are all absolute positions in scaled units.
  struct Range {
    double lower_limit;
    double upper_limit;
    double lower_hard_limit;
    double upper_hard_limit;
  };

  struct Shooter {
    double lower_limit;
    double upper_limit;
    double lower_hard_limit;
    double upper_hard_limit;
    // If the plunger is further back than this position, it is safe for the
    // latch to be down.  Anything else would be considered a collision.
    double latch_max_safe_position;
    AnglePair plunger_back;
    AnglePair pusher_distal;
    AnglePair pusher_proximal;
    double zeroing_speed;
    double unload_speed;
  };

  Shooter shooter;

  struct Claws {
    double claw_zeroing_off_speed;
    double claw_zeroing_speed;
    double claw_zeroing_separation;

    // claw separation that would be considered a collision
    double claw_min_separation;
    double claw_max_separation;

    // We should never get closer/farther than these.
    double soft_min_separation;
    double soft_max_separation;

    // Three hall effects are known as front, calib and back
    typedef Values::AnglePair AnglePair;

    struct Claw {
      double lower_hard_limit;
      double upper_hard_limit;
      double lower_limit;
      double upper_limit;
      AnglePair front;
      AnglePair calibration;
      AnglePair back;
    };

    Claw upper_claw;
    Claw lower_claw;

    double claw_unimportant_epsilon;
    double start_fine_tune_pos;
    double max_zeroing_voltage;
  };
  Claws claw;

  // Has all the constants for the ShootAction class.
  struct ShooterAction {
    // Minimum separation required between the claws in order to be able to
    // shoot.
    double claw_shooting_separation;

    // Goal to send to the claw when opening it up in preparation for shooting;
    // should be larger than claw_shooting_separation so that we can shoot
    // promptly.
    double claw_separation_goal;
   };
  ShooterAction shooter_action;
};

// Creates (once) a Values instance for ::aos::network::GetTeamNumber() and
// returns a reference to it.
const Values &GetValues();

// Creates Values instances for each team number it is called with and returns
// them.
const Values &GetValuesForTeam(uint16_t team_number);

}  // namespace constants
}  // namespace y2014

#endif  // Y2014_CONSTANTS_H_
