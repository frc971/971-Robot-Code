#ifndef FRC971_CONTROL_LOOPS_GEAR_H_
#define FRC971_CONTROL_LOOPS_GEAR_H_

namespace frc971 {
namespace control_loops {
namespace drivetrain {

// The state of the shift.
enum class Gear { HIGH, LOW, SHIFTING_UP, SHIFTING_DOWN };

// True if the the robot might or is trying to be in high gear.
inline bool MaybeHigh(Gear g) {
  return g == Gear::HIGH || g == Gear::SHIFTING_UP;
}

// True if the shifter is engaged and ready for action.
inline bool IsInGear(Gear gear) {
  return gear == Gear::LOW || gear == Gear::HIGH;
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_GEAR_H_
