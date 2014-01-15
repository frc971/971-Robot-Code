#ifndef AOS_LINUX_CODE_OUTPUT_MOTOR_OUTPUT_H_
#define AOS_LINUX_CODE_OUTPUT_MOTOR_OUTPUT_H_

#include <stdint.h>
#include <string.h>
#include <algorithm>
#include <string>

#include "aos/common/network/SendSocket.h"
#include "aos/common/byteorder.h"
#include "aos/common/type_traits.h"
#include "aos/externals/WPILib/WPILib/NetworkRobot/NetworkRobotValues.h"

namespace aos {

// A class for sending output values to a cRIO.
// values_ gets completely reset each time through, so RunIteration() needs to
// set everything each time.
class MotorOutput {
 public:
  MotorOutput();
  void Run();

  // A container for holding the constants that WPILib uses for each type of
  // motor controller to map floating point values to 1-byte PWM values to
  // actually use.
  struct MotorControllerBounds {
    // What 1.0 maps to.
    const uint8_t kMax;
    // The smallest value to map a positive speed to.
    const uint8_t kDeadbandMax;
    // What 0.0 maps to.
    const uint8_t kCenter;
    // The biggest value to map a negative speed to.
    const uint8_t kDeadbandMin;
    // What -1.0 maps to.
    const uint8_t kMin;

    // Applies the mapping.
    uint8_t Map(double value) const;
  };

 protected:
  // Brian got the values here by trying values with hardware on 11/23/12.
  static const MotorControllerBounds kTalonBounds;
  // Taken from WPILib.
  static const MotorControllerBounds kVictorBounds;

  // Helper methods for filling out values_.
  // All channels are the 1-indexed numbers that usually go into WPILib.
  void SetSolenoid(uint8_t channel, bool set);
  void SetPWMOutput(uint8_t channel, double value,
                    const MotorControllerBounds &bounds);
  void DisablePWMOutput(uint8_t channel);
  void SetDigitalOutput(uint8_t channel, bool value);

  // The data that's going to get sent over.
  // Gets reset (everything set so that it won't do anything) each time through.
  NetworkRobotMotors values_;

 private:
  // Subclasses need to actually fill out values_ here.
  virtual void RunIteration() = 0;

  SendSocket socket_;
};

}  // namespace aos

#endif
