#include "frc971/wpilib/buffered_pcm.h"

#include <inttypes.h>

#include "aos/common/logging/logging.h"

namespace frc971 {
namespace wpilib {

::std::unique_ptr<BufferedSolenoid> BufferedPcm::MakeSolenoid(int number) {
  return ::std::unique_ptr<BufferedSolenoid>(
      new BufferedSolenoid(number, this));
}

void BufferedPcm::Set(int number, bool value) {
  if (value) {
    values_ |= 1 << number;
  } else {
    values_ &= ~(1 << number);
  }
}

void BufferedPcm::Flush() {
  LOG(DEBUG, "sending solenoids 0x%" PRIx8 "\n", values_);
#ifdef WPILIB2017
  SolenoidBase::SetAll(values_, m_moduleNumber);
#else
  SolenoidBase::Set(values_, 0xFF, m_moduleNumber);
#endif
}

}  // namespace wpilib
}  // namespace frc971
