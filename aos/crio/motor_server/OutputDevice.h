#ifndef __CRIO_MOTOR_SERVER_OUTPUT_DEVICE_H_
#define __CRIO_MOTOR_SERVER_OUTPUT_DEVICE_H_

#include <stdint.h>
#include "aos/crio/shared_libs/ByteBuffer.h"

namespace aos {

class OutputDevice {
 protected:
  OutputDevice() {
  }
 public:
  // Reads the value out of buff and stores it somewhere for SetValue to use.
  // Returns whether or not it successfully read a whole value out of buff.
  virtual bool ReadValue(ByteBuffer &buff) = 0;
  // Actually sets the output device to the value saved by ReadValue.
  virtual void SetValue() = 0;
  // Gets called when no values come in for a while.
  virtual void NoValue() = 0;
};

} // namespace aos

#endif

