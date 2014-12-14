#include "frc971/wpilib/buffered_solenoid.h"

#include "frc971/wpilib/buffered_pcm.h"

namespace frc971 {
namespace wpilib {

void BufferedSolenoid::Set(bool value) {
  pcm_->Set(number_, value);
}

}  // namespace wpilib
}  // namespace frc971
