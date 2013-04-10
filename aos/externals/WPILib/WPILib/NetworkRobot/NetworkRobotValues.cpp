/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "WPILib/NetworkRobot/NetworkRobotValues.h"

#include <string.h>

namespace {

uint32_t CalculateHashValue(const char *data, size_t size) {
  uint32_t hash_value = 0;
  for (size_t i = 0; i < size; ++i) {
    hash_value = (((hash_value << 8) & 0xFF00) + (data[i] & 0xFF)) % 0x04C11DB7;
  }
  return hash_value;
}

}  // namespace

void NetworkRobotValues::FillInHashValue() {
  hash_value = CalculateHashValue(reinterpret_cast<const char *>(this) +
                                  sizeof(hash_value),
                                  sizeof(*this) - sizeof(hash_value));
}

bool NetworkRobotValues::CheckHashValue() const {
  return hash_value == CalculateHashValue(reinterpret_cast<const char *>(this) +
                                          sizeof(hash_value),
                                          sizeof(*this) - sizeof(hash_value));
}
