#include "frc971/wpilib/gyro_interface.h"

#include <inttypes.h>
#include <chrono>

#include "aos/common/logging/logging.h"
#include "aos/common/time.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace frc971 {
namespace wpilib {

GyroInterface::GyroInterface() : gyro_(new SPI(SPI::kOnboardCS0)) {
  // The gyro goes up to 8.08MHz.
  // The myRIO goes up to 4MHz, so the roboRIO probably does too.
  gyro_->SetClockRate(4e6);
  gyro_->SetChipSelectActiveLow();
  gyro_->SetClockActiveHigh();
  gyro_->SetSampleDataOnRising();
  gyro_->SetMSBFirst();
}

bool GyroInterface::InitializeGyro() {
  uint32_t result;
  if (!DoTransaction(0x20000003, &result)) {
    LOG(WARNING, "failed to start a self-check\n");
    return false;
  }
  if (result != 1) {
    // We might have hit a parity error or something and are now retrying, so
    // this isn't a very big deal.
    LOG(INFO, "gyro unexpected initial response 0x%" PRIx32 "\n", result);
  }

  // Wait for it to assert the fault conditions before reading them.
  ::std::this_thread::sleep_for(::std::chrono::milliseconds(50));

  if (!DoTransaction(0x20000000, &result)) {
    LOG(WARNING, "failed to clear latched non-fault data\n");
    return false;
  }
  LOG(DEBUG, "gyro dummy response is 0x%" PRIx32 "\n", result);

  if (!DoTransaction(0x20000000, &result)) {
    LOG(WARNING, "failed to read self-test data\n");
    return false;
  }
  if (ExtractStatus(result) != 2) {
    LOG(WARNING, "gyro first value 0x%" PRIx32 " not self-test data\n", result);
    return false;
  }
  if (ExtractErrors(result) != 0x7F) {
    LOG(WARNING, "gyro first value 0x%" PRIx32 " does not have all errors\n",
        result);
    return false;
  }

  if (!DoTransaction(0x20000000, &result)) {
    LOG(WARNING, "failed to clear latched self-test data\n");
    return false;
  }
  if (ExtractStatus(result) != 2) {
    LOG(WARNING, "gyro second value 0x%" PRIx32 " not self-test data\n",
        result);
    return false;
  }

  return true;
}

bool GyroInterface::DoTransaction(uint32_t to_write, uint32_t *result) {
  static const uint8_t kBytes = 4;
  static_assert(kBytes == sizeof(to_write),
                "need the same number of bytes as sizeof(the data)");

  if (__builtin_parity(to_write & ~1) == 0) {
    to_write |= 1;
  } else {
    to_write &= ~1;
  }

  uint8_t to_send[kBytes], to_receive[kBytes];
  const uint32_t to_write_flipped = __builtin_bswap32(to_write);
  memcpy(to_send, &to_write_flipped, kBytes);

  switch (gyro_->Transaction(to_send, to_receive, kBytes)) {
    case -1:
      LOG(INFO, "SPI::Transaction failed\n");
      return false;
    case kBytes:
      break;
    default:
      LOG(FATAL, "SPI::Transaction returned something weird\n");
  }

  memcpy(result, to_receive, kBytes);
  if (__builtin_parity(*result & 0xFFFF) != 1) {
    LOG(INFO, "high byte parity failure\n");
    return false;
  }
  if (__builtin_parity(*result) != 1) {
    LOG(INFO, "whole value parity failure\n");
    return false;
  }

  *result = __builtin_bswap32(*result);
  return true;
}

uint16_t GyroInterface::DoRead(uint8_t address) {
  const uint32_t command = (0x8 << 28) | (address << 17);
  uint32_t response;
  while (true) {
    if (!DoTransaction(command, &response)) {
      LOG(WARNING, "reading 0x%" PRIx8 " failed\n", address);
      continue;
    }
    if ((response & 0xEFE00000) != 0x4E000000) {
      LOG(WARNING, "gyro read from 0x%" PRIx8
                   " gave unexpected response 0x%" PRIx32 "\n",
          address, response);
      continue;
    }
    return (response >> 5) & 0xFFFF;
  }
}

double GyroInterface::ExtractAngle(uint32_t value) {
  const int16_t reading = -static_cast<int16_t>(value >> 10 & 0xFFFF);
  return static_cast<double>(reading) * 2.0 * M_PI / 360.0 / 80.0;
}

uint32_t GyroInterface::ReadPartID() {
  return (DoRead(0x0E) << 16) | DoRead(0x10);
}

uint32_t GyroInterface::GetReading() {
  uint32_t result;
  if (!DoTransaction(0x20000000, &result)) {
    return 0;
  }
  return result;
}

}  // namespace wpilib
}  // namespace frc971
