#ifndef AOS_CRIO_MOTOR_SERVER_MOTOR_CONTROLLER_OUTPUT_H_
#define AOS_CRIO_MOTOR_SERVER_MOTOR_CONTROLLER_OUTPUT_H_

#include "aos/crio/motor_server/OutputDevice.h"

#include "aos/crio/Talon.h"

#include "WPILib/SpeedController.h"
#include "WPILib/Jaguar.h"
#include "WPILib/CANJaguar.h"
#include "WPILib/Victor.h"

namespace aos {

// LinearizedVictor is a Victor that transforms the set values to linearize the
// hardware's response curve.
class LinearizedVictor : public Victor {
 public:
  explicit LinearizedVictor(uint32_t channel) : Victor(channel), speed_(0) {}
  virtual void Set(float speed, UINT8 syncGroup=0);
  virtual float Get();
  virtual void Disable();

  // Returns the linearized motor power to apply to get the motor to go at the
  // provided goal_speed.
  static double Linearize(double goal_speed);

 private:
  // The speed last sent to the Victor.
  float speed_;
};

class MotorControllerOutput : public OutputDevice {
 private:
  SpeedController &output;
 protected:
  double value;
  MotorControllerOutput(SpeedController *output) : OutputDevice(), output(*output) {
    value = 0.0;
  }
  // TODO(brians) add virtual destructor?

  virtual bool ReadValue(ByteBuffer &buff);
  virtual void SetValue();
  virtual void NoValue();
};

class JaguarOutput : public MotorControllerOutput {
 public:
  JaguarOutput(uint32_t port) : MotorControllerOutput(new Jaguar(port)) {}
};
class CANJaguarOutput : public MotorControllerOutput {
 public:
  CANJaguarOutput(uint32_t port) : MotorControllerOutput(new CANJaguar(port)) {}
};
class VictorOutput : public MotorControllerOutput {
 public:
  VictorOutput(uint32_t port) 
      : MotorControllerOutput(new LinearizedVictor(port)) {}
};
class TalonOutput : public MotorControllerOutput {
 public:
  TalonOutput(uint32_t port) : MotorControllerOutput(new Talon(port)) {}
};

}  // namespace aos

#endif
