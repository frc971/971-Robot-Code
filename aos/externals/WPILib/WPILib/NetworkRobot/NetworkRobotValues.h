/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef WPILIB_NETWORK_ROBOT_NETWORK_ROBOT_VALUES_H_
#define WPILIB_NETWORK_ROBOT_NETWORK_ROBOT_VALUES_H_

#include <stdint.h>
#include <arpa/inet.h>

// This file needs to not have any dependencies on any other parts of WPILib so
// that it can be #included by other code (like that which sends these values).
// All multi-byte values are sent over the network in big endian (network)
// byte order.
//
// The structures get run through a Serialize phase to avoid byte-order,
// padding, etc compatibility problems.

// Provides a convenient way to serialize/deserialize data.
// The serialized format consists of a 4-byte size (of everything, including the
// size and hash value), a 4-byte hash value, and then all of the actual data.
namespace buffers {
// The number of extra bytes needed on top of what the actual data takes.
// Code should use this constant in case other information is added to the
// start (or end).
static const size_t kOverhead = 4 + 4;

// Allows reading data out of a buffer.
// Instances are not safe for concurrent use.
//
// To make writing code easier, the Read* methods simply return 0 on overrun.
// Users should check at the end (see overrun()) to see if all of the data that
// they read is valid.
class Read {
 public:
  // Does not take ownership of data, but the object will use it throughout its
  // lifetime.
  Read(const char *data, size_t data_size)
      : data_(data), data_size_(data_size), current_index_(kOverhead) {}

  // Returns whether there is enough data according to the size recorded with
  // the data and the hash value is correct.
  // Will reset the current read position to the beginning of the data unless
  // overrun() is true.
  bool Check();

  // Returns whether or not the correct amount of data (according to the size
  // stored at the start) was read.
  // Will reset the current read position to the beginning of the data unless
  // overrun() is true.
  bool ReadCorrectAmount();

  uint8_t Read8() {
    current_index_ += 1;
    if (overrun()) return 0;
    return data_[current_index_ - 1];
  }
  uint16_t Read16() {
    current_index_ += 2;
    if (overrun()) return 0;
    union {
      uint16_t value;
      uint8_t bytes[2];
    } value;
    value.bytes[0] = data_[current_index_ - 2];
    value.bytes[1] = data_[current_index_ - 1];
    return ntohs(value.value);
  }
  uint32_t Read32() {
    current_index_ += 4;
    if (overrun()) return 0;
    union {
      uint32_t value;
      uint8_t bytes[4];
    } value;
    value.bytes[0] = data_[current_index_ - 4];
    value.bytes[1] = data_[current_index_ - 3];
    value.bytes[2] = data_[current_index_ - 2];
    value.bytes[3] = data_[current_index_ - 1];
    return ntohl(value.value);
  }

  // Returns whether or not we ran over the end.
  bool overrun() const { return current_index_ > data_size_; }

 private:
  // Where we are reading or writing from. Not owned by this object.
  const char *const data_;
  const size_t data_size_;

  // The index of the next read/write to data_.
  size_t current_index_;
};

// Allows writing data into a buffer.
// Instances are not safe for concurrent use.
//
// To make writing code easier, the Write* methods simply do nothing on overrun.
// Users should check at the end (see overrun()) to see if all of the data that
// they write actually got written.
class Write {
 public:
  Write(char *data, size_t data_size)
      : data_(data), data_size_(data_size), current_index_(kOverhead) {}

  // Fills in the hash value and size at the beginning and returns the number of
  // bytes used.
  // Afterwards, further writes will go to the beginning of the data again.
  // Returns the number of bytes used (including kOverhead at the beginning) or
  // -1 if writing too much data was attempted.
  ssize_t Finalize();

  void Write8(uint8_t value) {
    current_index_ += 1;
    if (overrun()) return;
    data_[current_index_ - 1] = value;
  }
  void Write16(uint16_t value) {
    current_index_ += 2;
    if (overrun()) return;
    union {
      uint16_t value;
      uint8_t bytes[2];
    } flipped;
    flipped.value = htons(value);
    data_[current_index_ - 2] = flipped.bytes[0];
    data_[current_index_ - 1] = flipped.bytes[1];
  }
  void Write32(uint32_t value) {
    current_index_ += 4;
    if (overrun()) return;
    union {
      uint32_t value;
      uint8_t bytes[4];
    } flipped;
    flipped.value = htonl(value);
    data_[current_index_ - 4] = flipped.bytes[0];
    data_[current_index_ - 3] = flipped.bytes[1];
    data_[current_index_ - 2] = flipped.bytes[2];
    data_[current_index_ - 1] = flipped.bytes[3];
  }

  // Returns whether or not we ran over the end.
  bool overrun() const { return current_index_ >= data_size_; }

 private:
  // Where we are reading or writing from. Not owned by this object.
  char *const data_;
  const size_t data_size_;

  // The index of the next read/write to data_.
  size_t current_index_;
};

}  // namespace buffers

// Contains motor and other output values.
// All channel and module numbers are 1-based like usual.
struct NetworkRobotMotors {
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

  // Serializes this object into data (which must be a buffer with at least
  // data_size bytes in it).
  // data_size should be at least sizeof(*this) + buffers::kOverhead to make sure
  // that it is big enough.
  // See DeserializeFrom to decode data back into an instance of this class.
  // Returns the number of bytes of data that were used or -1 if data_size is
  // too small.
  ssize_t SerializeTo(char *data, size_t data_size) const;
  // Deserializes data into this object (which must be a buffer of at least
  // data_size bytes).
  // data should be (all of) the data written by SerializeTo.
  // Returns whether or not the hash value etc information in data matched (if
  // false, the members of this object may or may not have been modified).
  bool DeserializeFrom(const char *data, size_t data_size);
};

// The structure that the cRIO sends out with joystick positions etc.
struct NetworkRobotJoysticks {
  // A structure that stores the information about a joystick and instances for
  // each of the 4 joysticks.
  struct Joystick {
    // Bitmask of the button values.
    // The LSB is button 1 and only a maximum of 12 are supported.
    uint16_t buttons;
    // Raw values for each of the 6 joystick axes.
    // The range of values depends on the joystick.
    int8_t axes[6];
  } joysticks[4];

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
    bool test_mode() const { return GetBit(kTestMode); }
    void set_test_mode(bool value) { SetBit(kTestMode, value); }
    bool fms_attached() const { return GetBit(kFmsAttached); }
    void set_fms_attached(bool value) { SetBit(kFmsAttached, value); }
    bool autonomous() const { return GetBit(kAutonomous); }
    void set_autonomous(bool value) { SetBit(kAutonomous, value); }
    bool enabled() const { return GetBit(kEnabled); }
    void set_enabled(bool value) { SetBit(kEnabled, value); }

   private:
    // Constants for which bit is which.
    static const int kTestMode = 0;
    static const int kFmsAttached = 1;
    static const int kAutonomous = 2;
    static const int kEnabled = 3;

    bool GetBit(int bit) const {
      return bits & (1 << bit);
    }
    void SetBit(int bit, bool value) {
      uint8_t mask = 1 << bit;
      bits &= ~mask;
      bits |= (mask & (value ? 0xFF : 0x00));
    }

    // So that it can access bits directly for Serialize/Deserialize.
    friend class NetworkRobotJoysticks;

    uint8_t bits;
  } control;

  // Serializes this object into data (which must be a buffer with at least
  // data_size bytes in it).
  // data_size should be at least sizeof(*this) + buffers::kOverhead to make sure
  // that it is big enough.
  // See DeserializeFrom to decode data back into an instance of this class.
  // Returns the number of bytes of data that were used or -1 if data_size is
  // too small.
  ssize_t SerializeTo(char *data, size_t data_size) const;
  // Deserializes data into this object (which must be a buffer of at least
  // data_size bytes).
  // data should be (all of) the data written by SerializeTo.
  // Returns whether or not the hash value etc information in data matched (if
  // false, the members of this object may or may not have been modified).
  bool DeserializeFrom(const char *data, size_t data_size);
};

#endif  // WPILIB_NETWORK_ROBOT_NETWORK_ROBOT_VALUES_H_
