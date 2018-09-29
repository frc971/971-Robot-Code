#include "frc971/wpilib/buffered_pcm.h"

#include <inttypes.h>

#include <HAL/HAL.h>
#include <HAL/Ports.h>
#include <HAL/Solenoid.h>
#include "aos/logging/logging.h"

namespace frc971 {
namespace wpilib {

BufferedPcm::BufferedPcm(int module) : module_(module) {
  for (int i = 0; i < 8; ++i) {
    int32_t status = 0;
    solenoid_handles_[i] =
        HAL_InitializeSolenoidPort(HAL_GetPortWithModule(module_, i), &status);
    if (status != 0) {
      LOG(FATAL, "Status was %d\n", status);
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
  int32_t result = HAL_GetAllSolenoids(module_, &status);
  if (status != 0) {
    LOG(ERROR, "Failed to flush, %d\n", status);
    return 0;
  }
  return result;
}

void BufferedPcm::Flush() {
  LOG(DEBUG, "sending solenoids 0x%" PRIx8 "\n", values_);
  int32_t status = 0;
  HAL_SetAllSolenoids(module_, static_cast<int>(values_), &status);
  if (status != 0) {
    LOG(ERROR, "Failed to flush, %d\n", status);
  }
}

}  // namespace wpilib
}  // namespace frc971
