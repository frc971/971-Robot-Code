#include <cmath>

#include "frc971/zeroing/wrap.h"
#include "y2019/joystick_angle.h"

namespace y2019 {
namespace input {
namespace joysticks {

using ::frc971::zeroing::Wrap;

bool AngleCloseTo(double angle, double near, double range) {
  double wrapped_angle = Wrap(near, angle, 2 * M_PI);

  return ::std::abs(wrapped_angle - near) < range;
}

JoystickAngle GetJoystickPosition(const JoystickAxis &x_axis,
                                  const JoystickAxis &y_axis,
                                  const Data &data) {
  return GetJoystickPosition(data.GetAxis(x_axis), data.GetAxis(y_axis));
}

JoystickAngle GetJoystickPosition(float x_axis, float y_axis) {
  const float magnitude = hypot(x_axis, y_axis);

  if (magnitude < 0.5) {
    return JoystickAngle::kDefault;
  }

  double angle = atan2(y_axis, x_axis);

  if (AngleCloseTo(angle, M_PI / 3, M_PI / 6)) {
    return JoystickAngle::kUpperRight;
  } else if (AngleCloseTo(angle, 2 * M_PI / 3, M_PI / 6)) {
    return JoystickAngle::kUpperLeft;
  } else if (AngleCloseTo(angle, M_PI, M_PI / 6)) {
    return JoystickAngle::kMiddleLeft;
  } else if (AngleCloseTo(angle, 0, M_PI / 6)) {
    return JoystickAngle::kMiddleRight;
  } else if (AngleCloseTo(angle, -M_PI / 3, M_PI / 6)) {
    return JoystickAngle::kLowerRight;
  } else if (AngleCloseTo(angle, -2 * M_PI / 3, M_PI / 6)) {
    return JoystickAngle::kLowerLeft;
  }

  return JoystickAngle::kDefault;
}
}  // namespace joysticks
}  // namespace input
}  // namespace y2019
