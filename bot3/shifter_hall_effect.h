#ifndef BOT3_SHIFTER_HALL_EFFECT_H_
#define BOT3_SHIFTER_HALL_EFFECT_H_

namespace bot3 {
namespace constants {

// Contains the voltages for an analog hall effect sensor on a shifter.
struct ShifterHallEffect {
  // The numbers to use for scaling raw voltages to 0-1.
  double high, low;

  // The numbers for when the dog is clear of each gear.
  double clear_high, clear_low;
};


} // constants
} // bot3

#endif
