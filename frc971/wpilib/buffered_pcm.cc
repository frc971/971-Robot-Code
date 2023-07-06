#include "frc971/wpilib/buffered_pcm.h"

#include <cinttypes>

#include "aos/logging/logging.h"
#include "hal/CTREPCM.h"
#include "hal/HAL.h"
#include "hal/Ports.h"

namespace frc971 {
namespace wpilib {

BufferedPcm::BufferedPcm(int module) : module_(module) {
  for (int i = 0; i < 8; ++i) {
    int32_t status = 0;
    solenoid_handles_[i] = HAL_InitializeCTREPCM(
        HAL_GetPortWithModule(module_, i), nullptr, &status);
    if (status != 0) {
      AOS_LOG(FATAL, "Status was %d\n", status);
    }
  }
}

::std::unique_ptr<BufferedSolenoid> BufferedPcm::MakeSolenoid(int number) {
  return ::std::unique_ptr<BufferedSolenoid>(
      new BufferedSolenoid(number, this));
}

void BufferedPcm::DoSet(int number, bool value) {
  if (value) {
    values_ |= 1 << number;
  } else {
    values_ &= ~(1 << number);
  }
}

int32_t BufferedPcm::GetAll() {
  int32_t status = 0;
  int32_t result = HAL_GetCTREPCMSolenoids(module_, &status);
  if (status != 0) {
    AOS_LOG(ERROR, "Failed to flush, %d\n", status);
    return 0;
  }
  return result;
}

void BufferedPcm::Flush() {
  AOS_LOG(DEBUG, "sending solenoids 0x%" PRIx8 "\n", values_);
  int32_t status = 0;
  HAL_SetCTREPCMSolenoids(module_, 0xff, static_cast<int>(values_), &status);
  if (status != 0) {
    AOS_LOG(ERROR, "Failed to flush, %d\n", status);
  }
}

}  // namespace wpilib
}  // namespace frc971
