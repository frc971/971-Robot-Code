#ifndef FRC971_SHIFTER_HALL_EFFECT_H_
#define FRC971_SHIFTER_HALL_EFFECT_H_

namespace frc971 {
namespace constants {

// Contains the voltages for an analog hall effect sensor on a shifter.
struct ShifterHallEffect {
  // The numbers to use for scaling raw voltages to 0-1.
  // Low is near 0.0, high is near 1.0
  double low_gear_middle, low_gear_low;
  double high_gear_high, high_gear_middle;

  // The numbers for when the dog is clear of each gear.
  double clear_low, clear_high;
};

} // constants
} // frc971

#endif
