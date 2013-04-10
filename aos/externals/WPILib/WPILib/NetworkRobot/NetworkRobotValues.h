/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef WPILIB_NETWORK_ROBOT_NETWORK_ROBOT_VALUES_H_
#define WPILIB_NETWORK_ROBOT_NETWORK_ROBOT_VALUES_H_

#include <stdint.h>

// This file needs to not have any dependencies on any other parts of WPILib so
// that it can be #included by other code (like that which sends these values).

// The structure that actually gets sent over the network.
// All multi-byte values are sent over the network in big endian (network)
// byte order.
// All channel and module numbers are 1-based like usual.
struct NetworkRobotValues {
  // A hash value to make sure that corrupted packets don't get used.
  // IMPORTANT: Must stay at the beginning.
  uint32_t hash_value;

  // Which digital module this packet has values for (1 or 2).
  // 0 means the default one and -1 means to not update any one.
  int8_t digital_module;
  // Raw values for all 10 PWM outputs.
  uint8_t pwm_outputs[10];

  // Bitmasks for enabling digital outputs and what values to set them to.
  // See DigitalModule::SetDIOs for which bit is which.
  uint16_t digital_output_enables;
  uint16_t digital_output_values;

  // Channels for a presssure switch and compressor to turn on and off based on
  // the value from it.
  // Whatever compressor channel is sent will be updated with the value from the
  // corresponding pressure switch channel when the packet is received.
  // 0 means to not do anything.
  uint8_t pressure_switch_channel, compressor_channel;

  // Which solenoid module this packet has values for (1 or 2).
  // 0 means the default one and -1 means to not update any one.
  int8_t solenoid_module;
  // 1 bit for each solenoid output.
  uint8_t solenoid_values;

  // Sets hash_value to the correct value for the rest of the data.
  void FillInHashValue();
  // Returns whether or not hash_value matches the rest of the data. Any byte
  // order conversion must be performed ONLY on the hash_value field before
  // calling this.
  bool CheckHashValue() const;
} __attribute__((packed));

#endif  // WPILIB_NETWORK_ROBOT_NETWORK_ROBOT_VALUES_H_
