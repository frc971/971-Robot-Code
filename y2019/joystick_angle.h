#ifndef Y2019_JOYSTICK_ANGLE_H_
#define Y2019_JOYSTICK_ANGLE_H_

#include "aos/input/driver_station_data.h"

using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::Data;

namespace y2019 {
namespace input {
namespace joysticks {
bool AngleCloseTo(double angle, double near, double range);

enum class JoystickAngle {
  kDefault,
  kUpperRight,
  kMiddleRight,
  kLowerRight,
  kUpperLeft,
  kMiddleLeft,
  kLowerLeft
};

JoystickAngle GetJoystickPosition(const JoystickAxis &x_axis,
                                  const JoystickAxis &y_axis, const Data &data);
JoystickAngle GetJoystickPosition(float x_axis, float y_axis);

}  // namespace joysticks
}  // namespace input
}  // namespace y2019

#endif  // Y2019_JOYSTICK_ANGLE_H_
