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
// All multi-byte values are sent over the network in big endian (network)
// byte order.

// The structure that actually gets sent over the network to the cRIO with motor
// values.
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

// The structure that the cRIO sends out with joystick positions etc.
struct NetworkRobotJoysticks {
  // A hash value to make sure that corrupted packets don't get used.
  // IMPORTANT: Must stay at the beginning.
  uint32_t hash_value;

  // A structure that stores the information about a joystick and instances for
  // each of the 4 joysticks.
  struct Joystick {
    // Bitmask of the button values.
    // The LSB is button 1 and only a maximum of 12 are supported.
    uint16_t buttons;
    // Raw values for each of the 6 joystick axes.
    // The range of values depends on the joystick.
    int8_t axes[6];
  } __attribute__((packed)) joysticks[4];

  // The index number from the DS.
  uint16_t control_packet_index;
  // An index for this structure. Gets incremented by 1 for each one of these
  // structures that is sent.
  uint16_t index;

  // The team number that the DS is configured for.
  uint16_t team_number;
  // Which alliance the robot is on. Should be 'R' or 'B'.
  char alliance;
  // Which position the DS is in on the field. Should be '1', '2', or '3'.
  char position;

  // A structure that efficiently stores the control data bits from the DS and
  // has methods for safely and easily getting and setting them and an instance
  // of it for actually sending the information.
  // Not just a C bitfield because those are a mess for portability.
  struct ControlInformation {
    bool test_mode() { return GetBit(kTestMode); }
    void set_test_mode(bool value) { SetBit(kTestMode, value); }
    bool fms_attached() { return GetBit(kFmsAttached); }
    void set_fms_attached(bool value) { SetBit(kFmsAttached, value); }
    bool autonomous() { return GetBit(kAutonomous); }
    void set_autonomous(bool value) { SetBit(kAutonomous, value); }
    bool enabled() { return GetBit(kEnabled); }
    void set_enabled(bool value) { SetBit(kEnabled, value); }

   private:
    // Constants for which bit is which.
    static const int kTestMode = 0;
    static const int kFmsAttached = 1;
    static const int kAutonomous = 2;
    static const int kEnabled = 3;

    bool GetBit(int bit) {
      return bits & (1 << bit);
    }
    void SetBit(int bit, bool value) {
      uint8_t mask = 1 << bit;
      bits &= ~mask;
      bits |= (mask & (value ? 0xFF : 0x00));
    }

    uint8_t bits;
  } __attribute__((packed)) control;

  // Sets hash_value to the correct value for the rest of the data.
  void FillInHashValue();
  // Returns whether or not hash_value matches the rest of the data. Any byte
  // order conversion must be performed ONLY on the hash_value field before
  // calling this.
  bool CheckHashValue() const;
} __attribute__((packed));

#endif  // WPILIB_NETWORK_ROBOT_NETWORK_ROBOT_VALUES_H_
