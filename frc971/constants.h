#ifndef FRC971_CONSTANTS_H_
#define FRC971_CONSTANTS_H_

#include <stdint.h>

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
  // Wrist hall effect positive and negative edges.
  // How many radians from horizontal to the location of interest.
  double wrist_hall_effect_start_angle;
  double wrist_hall_effect_stop_angle;

  // Upper and lower extreme limits of travel for the wrist.
  // These are the soft stops for up and down.
  double wrist_upper_limit;
  double wrist_lower_limit;

  // Physical limits.  These are here for testing.
  double wrist_upper_physical_limit;
  double wrist_lower_physical_limit;

  // Zeroing speed.
  // The speed to move the wrist at when zeroing in rad/sec
  double wrist_zeroing_speed;
  // Zeroing off speed (in rad/sec).
  double wrist_zeroing_off_speed;

  // AngleAdjust hall effect positive and negative edges.
  // These are the soft stops for up and down.
  const double (&angle_adjust_hall_effect_start_angle)[2];
  const double (&angle_adjust_hall_effect_stop_angle)[2];

  // Upper and lower extreme limits of travel for the angle adjust.
  double angle_adjust_upper_limit;
  double angle_adjust_lower_limit;
  // Physical limits.  These are here for testing.
  double angle_adjust_upper_physical_limit;
  double angle_adjust_lower_physical_limit;

  // The speed to move the angle adjust when zeroing, in rad/sec
  double angle_adjust_zeroing_speed;
  // Zeroing off speed.
  double angle_adjust_zeroing_off_speed;

  // Deadband voltage.
  double angle_adjust_deadband;

  // The number of teeth on the pinion that drives the drivetrain wheels.
  int drivetrain_gearbox_pinion;

  ShifterHallEffect left_drive, right_drive;

  // How many pixels off center the vertical line
  // on the camera view is.
  int camera_center;
};

// Creates (once) a Values instance and returns a reference to it.
const Values &GetValues();

}  // namespace constants
}  // namespace frc971

#endif  // FRC971_CONSTANTS_H_
