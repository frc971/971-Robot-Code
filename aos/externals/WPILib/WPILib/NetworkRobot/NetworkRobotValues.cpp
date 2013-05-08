/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "WPILib/NetworkRobot/NetworkRobotValues.h"

#include <string.h>

namespace buffers {
namespace {

uint32_t CalculateHashValue(const char *data, size_t size) {
  uint32_t hash_value = 0;
  for (size_t i = 0; i < size; ++i) {
    hash_value = (((hash_value << 8) & 0xFF00) + (data[i] & 0xFF)) % 0x04C11DB7;
  }
  return hash_value;
}

}  // namespace

bool Read::Check() {
  if (data_size_ < kOverhead) return false;

  current_index_ = 0;
  uint32_t size = Read32();
  uint32_t hash = Read32();

  if (data_size_ < size) return false;

  uint32_t expected = CalculateHashValue(data_ + kOverhead, size - kOverhead);
  return hash == expected;
}

bool Read::ReadCorrectAmount() {
  if (overrun()) return false;

  size_t read = current_index_;

  current_index_ = 0;
  uint32_t size = Read32();
  current_index_ = kOverhead;

  return read == size;
}

ssize_t Write::Finalize() {
  if (overrun()) return -1;

  uint32_t size = current_index_;
  uint32_t hash = CalculateHashValue(data_ + kOverhead, size - kOverhead);

  current_index_ = 0;
  Write32(size);
  Write32(hash);

  return size;
}

}  // namespace buffers

ssize_t NetworkRobotMotors::SerializeTo(char *data, size_t data_size) const {
  buffers::Write buffer(data, data_size);

  buffer.Write8(digital_module);
  for (size_t i = 0; i < sizeof(pwm_outputs) / sizeof(pwm_outputs[0]); ++i) {
    buffer.Write8(pwm_outputs[i]);
  }
  buffer.Write16(digital_output_enables);
  buffer.Write16(digital_output_values);
  buffer.Write8(pressure_switch_channel);
  buffer.Write8(compressor_channel);
  buffer.Write8(solenoid_module);
  buffer.Write8(solenoid_values);

  return buffer.Finalize();
}

bool NetworkRobotMotors::DeserializeFrom(const char *data, size_t data_size) {
  buffers::Read buffer(data, data_size);
  if (!buffer.Check()) return false;

  digital_module = buffer.Read8();
  for (size_t i = 0; i < sizeof(pwm_outputs) / sizeof(pwm_outputs[0]); ++i) {
    pwm_outputs[i] = buffer.Read8();
  }
  digital_output_enables = buffer.Read16();
  digital_output_values = buffer.Read16();
  pressure_switch_channel = buffer.Read8();
  compressor_channel = buffer.Read8();
  solenoid_module = buffer.Read8();
  solenoid_values = buffer.Read8();

  return buffer.ReadCorrectAmount();
}

ssize_t NetworkRobotJoysticks::SerializeTo(char *data, size_t data_size) const {
  buffers::Write buffer(data, data_size);

  for (size_t i = 0; i < sizeof(joysticks) / sizeof(joysticks[0]); ++i) {
    buffer.Write16(joysticks[i].buttons);
    for (size_t ii = 0;
         ii < sizeof(joysticks[0].axes) / sizeof(joysticks[0].axes[0]);
         ++ii) {
      buffer.Write8(joysticks[i].axes[ii]);
    }
  }
  buffer.Write16(control_packet_index);
  buffer.Write16(index);
  buffer.Write16(team_number);
  buffer.Write8(alliance);
  buffer.Write8(position);
  buffer.Write8(control.bits);

  return buffer.Finalize();
}

bool NetworkRobotJoysticks::DeserializeFrom(const char *data, size_t data_size) {
  buffers::Read buffer(data, data_size);
  if (!buffer.Check()) return false;

  for (size_t i = 0; i < sizeof(joysticks) / sizeof(joysticks[0]); ++i) {
    joysticks[i].buttons = buffer.Read16();
    for (size_t ii = 0;
         ii < sizeof(joysticks[0].axes) / sizeof(joysticks[0].axes[0]);
         ++ii) {
      joysticks[i].axes[ii] = buffer.Read8();
    }
  }
  control_packet_index = buffer.Read16();
  index = buffer.Read16();
  team_number = buffer.Read16();
  alliance = buffer.Read8();
  position = buffer.Read8();
  control.bits = buffer.Read8();

  return buffer.ReadCorrectAmount();
}
