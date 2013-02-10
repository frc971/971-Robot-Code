#ifndef __CRIO_MOTOR_SERVER_SOLENOID_OUTPUT_H_
#define __CRIO_MOTOR_SERVER_SOLENOID_OUTPUT_H_

#include "aos/crio/motor_server/OutputDevice.h"

namespace aos {

class SolenoidOutput : public OutputDevice {
 private:
  Solenoid solenoid;
  bool value;
 public:
  SolenoidOutput(uint32_t port) : OutputDevice(), solenoid(port), value(false) {
  }
 protected:
  virtual bool ReadValue(ByteBuffer &buff) {
    const int on = buff.read_char();
    if (on != 0 && on != 1) {
      if (on != -1) {
        LOG(ERROR, "illegal solenoid value %d\n", on);
      }
      return false;
    }
    value = on;
    return true;
  }
  virtual void SetValue() {
    solenoid.Set(value);
  }
  virtual void NoValue() {
    // leave the solenoid in its previous state
  }
};

} // namespace aos

#endif

