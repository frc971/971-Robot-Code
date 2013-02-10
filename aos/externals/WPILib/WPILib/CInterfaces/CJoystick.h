/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef _C_JOYSTICK_H_
#define _C_JOYSTICK_H_

static const UINT32 kDefaultXAxis = 1;
static const UINT32 kDefaultYAxis = 2;
static const UINT32 kDefaultZAxis = 3;
static const UINT32 kDefaultTwistAxis = 4;
static const UINT32 kDefaultThrottleAxis = 3;

typedef enum {
	kLeftHand = 0,
	kRightHand = 1
} JoystickHand;

typedef enum
{
	kXAxis, kYAxis, kZAxis, kTwistAxis, kThrottleAxis, kNumAxisTypes
} AxisType;
static const UINT32 kDefaultTriggerButton = 1;
static const UINT32 kDefaultTopButton = 2;
typedef enum
{
	kTriggerButton, kTopButton, kNumButtonTypes
} ButtonType;

UINT32 GetAxisChannel(UINT32 port, AxisType axis);
void SetAxisChannel(UINT32 port, AxisType axis, UINT32 channel); 

float GetX(UINT32 port, JoystickHand hand = kRightHand);
float GetY(UINT32 port, JoystickHand hand = kRightHand);
float GetZ(UINT32 port);
float GetTwist(UINT32 port);
float GetThrottle(UINT32 port);
float GetAxis(UINT32 port, AxisType axis);
float GetRawAxis(UINT32 port, UINT32 axis);

bool GetTrigger(UINT32 port, JoystickHand hand = kRightHand);
bool GetTop(UINT32 port, JoystickHand hand = kRightHand);
bool GetBumper(UINT32 port, JoystickHand hand = kRightHand);
bool GetButton(UINT32 port, ButtonType button);
bool GetRawButton(UINT32 port, UINT32 button);

#endif
 
