#include "y2019/joystick_angle.h"
#include <iostream>
#include "aos/input/driver_station_data.h"
#include "gtest/gtest.h"

using y2019::input::joysticks::JoystickAngle;
using y2019::input::joysticks::GetJoystickPosition;

TEST(JoystickAngleTest, JoystickAngleTest) {
  EXPECT_EQ(JoystickAngle::kUpperRight, GetJoystickPosition(0.75, 0.75));
  EXPECT_EQ(JoystickAngle::kMiddleRight, GetJoystickPosition(0.75, 0));
  EXPECT_EQ(JoystickAngle::kLowerRight, GetJoystickPosition(0.75, -0.75));
  EXPECT_EQ(JoystickAngle::kUpperLeft, GetJoystickPosition(-0.75, 0.75));
  EXPECT_EQ(JoystickAngle::kMiddleLeft, GetJoystickPosition(-0.75, 0));
  EXPECT_EQ(JoystickAngle::kLowerLeft, GetJoystickPosition(-0.75, -0.75));

  EXPECT_EQ(JoystickAngle::kDefault, GetJoystickPosition(0, 0));
}
