#ifndef AOS_INPUT_JOYSTICK_INPUT_H_
#define AOS_INPUT_JOYSTICK_INPUT_H_

#include "FRCComm.h"

namespace aos {

// Class for implementing atom code that reads the joystick values from the
// cRIO.
// Designed for a subclass that implements RunIteration to be instantiated and
// Runed.
class JoystickInput {
 private:
  uint16_t buttons[4], old_buttons[4];
  inline uint16_t MASK(int button) {
    return 1 << ((button > 8) ? (button - 9) : (button + 7));
  }
  void SetupButtons();
 protected:
  FRCCommonControlData control_data_;

  // Constants that retrieve data when used with joystick 0.
  static const int ENABLED = 13;
  static const int AUTONOMOUS = 14;
  static const int FMS_ATTACHED = 15;
  static const int TEST_MODE = 16;
  bool Pressed(int stick, int button) {
	  return buttons[stick] & MASK(button);
  }
  bool PosEdge(int stick, int button) {
	  return !(old_buttons[stick] & MASK(button)) && (buttons[stick] & MASK(button));
  }
  bool NegEdge(int stick, int button) {
	  return (old_buttons[stick] & MASK(button)) && !(buttons[stick] & MASK(button));
  }

  virtual void RunIteration() = 0;
 public:
  // Enters an infinite loop that reads values and calls RunIteration.
  void Run();
};

} // namespace aos

#endif

