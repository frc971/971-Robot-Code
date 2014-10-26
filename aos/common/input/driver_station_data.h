#ifndef AOS_COMMON_INPUT_DRIVER_STATION_DATA_H_
#define AOS_COMMON_INPUT_DRIVER_STATION_DATA_H_

// This file defines several types to support nicely looking at the data
// received from the driver's station.

#include <memory>

#include "aos/common/messages/robot_state.q.h"

namespace aos {
namespace input {
namespace driver_station {

// Represents a feature of a joystick (a button or an axis).
// All indices are 1-based.
class JoystickFeature {
 public:
  JoystickFeature(int joystick, int number)
      : joystick_(joystick), number_(number) {}

  // How many joysticks there are.
  static const int kJoysticks = sizeof(RobotState::joysticks) /
                                sizeof(RobotState::joysticks[0]);

  // Which joystick number this is (1-based).
  int joystick() const { return joystick_; }
  // Which feature on joystick() this is (1-based).
  int number() const { return number_; }

 private:
  const int joystick_, number_;
};

// Represents the location of a button.
// Use Data to actually get the value.
// Safe for static initialization.
class ButtonLocation : public JoystickFeature {
 public:
  ButtonLocation(int joystick, int number)
      : JoystickFeature(joystick, number) {}

  // How many buttons there are available on each joystick.
  static const int kButtons = 12;
};

// Represents various bits of control information that the DS sends.
// Use Data to actually get the value.
enum class ControlBit {
  kTestMode, kFmsAttached, kAutonomous, kEnabled
};

// Represents a single axis of a joystick.
// Use Data to actually get the value.
// Safe for static initialization.
class JoystickAxis : public JoystickFeature {
 public:
  JoystickAxis(int joystick, int number)
      : JoystickFeature(joystick, number) {}

  // How many axes there are available on each joystick.
  static const int kAxes = sizeof(Joystick::axis) /
                           sizeof(Joystick::axis[0]);
};

class Data {
 public:
  // Initializes the data to all buttons and control bits off and all joysticks
  // at 0.
  Data();

  // Updates the current information with a new set of values.
  void Update(const RobotState &new_values);

  bool IsPressed(ButtonLocation location) const;
  bool PosEdge(ButtonLocation location) const;
  bool NegEdge(ButtonLocation location) const;

  bool GetControlBit(ControlBit bit) const;
  bool PosEdge(ControlBit bit) const;
  bool NegEdge(ControlBit bit) const;

  // Returns the value in the range [-1.0, 1.0].
  float GetAxis(JoystickAxis axis) const;

 private:
  RobotState current_values_, old_values_;
};

}  // namespace driver_station
}  // namespace input
}  // namespace aos

#endif  // AOS_COMMON_INPUT_DRIVER_STATION_DATA_H_
