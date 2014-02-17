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
  struct Pair {
    double lower_limit;
    double upper_limit;
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
  
  struct ShooterLimits {
    double lower_limit;
    double upper_limit;
    Pair plunger_back;
    Pair pusher_distal;
    Pair pusher_proximal;
  };

  ShooterLimits shooter;
  
  double shooter_voltage;
  double shooter_total_length;
  double shooter_hall_effect_start_position;
  double shooter_zeroing_off_speed;
  double shooter_zeroing_speed;
  double position;

  struct Claws {
    double claw_zeroing_off_speed;
    double claw_zeroing_speed;
    double claw_zeroing_separation;

    // claw seperation that would be considered a collision
    double claw_min_seperation;
    double claw_max_seperation;

    // Three hall effects are known as front, calib and back
    struct AnglePair {
      double lower_angle;
      double upper_angle;
    };

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
  };
  Claws claw;
};

// Creates (once) a Values instance and returns a reference to it.
const Values &GetValues();

}  // namespace constants
}  // namespace frc971

#endif  // FRC971_CONSTANTS_H_
