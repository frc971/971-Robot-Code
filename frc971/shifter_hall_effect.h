#ifndef FRC971_SHIFTER_HALL_EFFECT_H_
#define FRC971_SHIFTER_HALL_EFFECT_H_

namespace frc971 {
namespace constants {

// Contains the constants for mapping the analog voltages that the shifter
// sensors return to the shifter position.  The code which uses this is trying
// to sort out if we are in low gear, high gear, or neutral.
struct ShifterHallEffect {
  // low_gear_low is the voltage that the shifter position sensor reads when it
  // is all the way in low gear.  high_gear_high is the voltage that the shifter
  // position sensor reads when it is all the way in high gear.  These two
  // values are used to calculate a position from 0 to 1, where we get 0 when
  // the shifter is in low gear, and 1 when it is high gear.
  double low_gear_low;
  double high_gear_high;

  // The numbers for when the dog is clear of each gear.
  // We are in low gear when the position is less than clear_low, and in high
  // gear when the shifter position is greater than clear_high.
  double clear_low, clear_high;
};

struct DualHallShifterHallEffect {
  ShifterHallEffect shifter_hall_effect;
  double low_gear_middle;
  double high_gear_middle;
};

}  // namespace constants
}  // namespace frc971

#endif
