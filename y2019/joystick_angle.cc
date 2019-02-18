#include "y2019/joystick_angle.h"

namespace y2019 {
namespace input {
namespace joysticks {

JoystickAngle GetJoystickPosition(const JoystickAxis &x_axis,
                                  const JoystickAxis &y_axis,
                                  const Data &data) {
  return GetJoystickPosition(data.GetAxis(x_axis), data.GetAxis(y_axis));
}

JoystickAngle GetJoystickPosition(float x_axis, float y_axis) {
  if (x_axis > kJoystickRight) {
    if (y_axis < kJoystickDown) {
      return JoystickAngle::kUpperRight;
    } else if (y_axis > kJoystickUp) {
      return JoystickAngle::kLowerRight;
    } else {
      return JoystickAngle::kMiddleRight;
    }
  } else if (x_axis < kJoystickLeft) {
    if (y_axis < kJoystickDown) {
      return JoystickAngle::kUpperLeft;
    } else if (y_axis > kJoystickUp) {
      return JoystickAngle::kLowerLeft;
    } else {
      return JoystickAngle::kMiddleLeft;
    }
  }
  return JoystickAngle::kDefault;
}

}  // namespace joysticks
}  // namespace input
}  // namespace y2019
