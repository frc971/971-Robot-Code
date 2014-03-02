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
  double drivetrain_done_distance;
  double drivetrain_max_speed;
};

// Creates (once) a Values instance and returns a reference to it.
const Values &GetValues();
const Values &GetValuesForTeam(uint16_t team_number);

}  // namespace constants
}  // namespace frc971

#endif  // FRC971_CONSTANTS_H_
