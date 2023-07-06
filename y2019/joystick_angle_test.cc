#include "y2019/joystick_angle.h"

#include <iostream>

#include "gtest/gtest.h"

#include "frc971/input/driver_station_data.h"

using y2019::input::joysticks::GetJoystickPosition;
using y2019::input::joysticks::JoystickAngle;

TEST(JoystickAngleTest, JoystickAngleTest) {
  EXPECT_EQ(JoystickAngle::kUpperRight, GetJoystickPosition(0.75, 0.75));
  EXPECT_EQ(JoystickAngle::kMiddleRight, GetJoystickPosition(0.75, 0));
  EXPECT_EQ(JoystickAngle::kLowerRight, GetJoystickPosition(0.75, -0.75));
  EXPECT_EQ(JoystickAngle::kUpperLeft, GetJoystickPosition(-0.75, 0.75));
  EXPECT_EQ(JoystickAngle::kMiddleLeft, GetJoystickPosition(-0.75, 0));
  EXPECT_EQ(JoystickAngle::kLowerLeft, GetJoystickPosition(-0.75, -0.75));

  EXPECT_EQ(JoystickAngle::kDefault, GetJoystickPosition(0, 0));
}
