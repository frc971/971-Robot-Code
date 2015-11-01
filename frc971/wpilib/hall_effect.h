#ifndef FRC971_WPILIB_HALL_EFFECT_H_
#define FRC971_WPILIB_HALL_EFFECT_H_

#include "DigitalInput.h"
#undef ERROR

namespace frc971 {
namespace wpilib {

// Inverts the output from a digital input.
class HallEffect : public DigitalInput {
 public:
  HallEffect(int index) : DigitalInput(index) {}
  virtual bool Get() override { return !DigitalInput::Get(); }
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_HALL_EFFECT_H_
