#ifndef FRC971_WPILIB_HALL_EFFECT_H_
#define FRC971_WPILIB_HALL_EFFECT_H_

#include "DigitalInput.h"

namespace frc971 {
namespace wpilib {

class HallEffect : public DigitalInput {
 public:
  HallEffect(int index) : DigitalInput(index) {}
  bool GetHall() { return !Get(); }
};

}  // namespace wpilib
}  // namespace frc971

#endif // FRC971_WPILIB_HALL_EFFECT_H_
