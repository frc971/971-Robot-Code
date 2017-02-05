#ifndef AOS_INPUT_JOYSTICK_INPUT_H_
#define AOS_INPUT_JOYSTICK_INPUT_H_

#include <atomic>

#include "aos/common/input/driver_station_data.h"

namespace aos {
namespace input {

// A class for handling joystick packet values.
// It will call RunIteration each time a new packet is received.
//
// This class automatically handles updating ::aos::joystick_state and logging
// (at INFO) button edges.
class JoystickInput {
 public:
  void Run();

 private:
  // Subclasses should do whatever they want with data here.
  virtual void RunIteration(const driver_station::Data &data) = 0;

  static void Quit(int /*signum*/);

  static ::std::atomic<bool> run_;
};

// Class which will proxy joystick information from UDP packets to the queues.
class JoystickProxy {
 public:
  void Run();
};

}  // namespace input
}  // namespace aos

#endif  // AOS_INPUT_JOYSTICK_INPUT_H_
