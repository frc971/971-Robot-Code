/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "Joystick.h"
#include "CInterfaces/CJoystick.h"

static Joystick *joysticks[4];
static bool initialized = false;

/**
 * Get the joystick associated with a port.
 * An internal function that will return the joystick object associated with a given
 * joystick port number. On the first call, all four joysticks are preallocated.
 *
 * @param port The joystick (USB) port number
 */
static Joystick *getJoystick(UINT32 port)
{
	if (!initialized)
	{
		for (int i = 0; i < 4; i++)
		{
			joysticks[i] = new Joystick(i+1);
		}
		initialized = true;
	}
	if (port < 1 || port > 4) return NULL;
	return joysticks[port - 1];
}


/**
 * Get the channel currently associated with the specified axis.
 *
 * @param port The USB port for this joystick.
 * @param axis The axis to look up the channel for.
 * @return The channel fr the axis.
 */
UINT32 GetAxisChannel(UINT32 port, AxisType axis)
{
	Joystick *stick = getJoystick(port);
	if (stick == NULL) return 0;
	return stick->GetAxisChannel((Joystick::AxisType) axis);
}

/**
 * Set the channel associated with a specified axis.
 *
 * @param port The USB port for this joystick.
 * @param axis The axis to set the channel for.
 * @param channel The channel to set the axis to.
 */
void SetAxisChannel(UINT32 port, AxisType axis, UINT32 channel)
{
	Joystick *stick = getJoystick(port);
	if (stick == NULL) return;
	stick->SetAxisChannel((Joystick::AxisType) axis, channel);
}

/**
 * Get the X value of the joystick.
 * This depends on the mapping of the joystick connected to the current port.
 *
 * @param port The USB port for this joystick.
 */
float GetX(UINT32 port, JoystickHand hand)
{
	Joystick *stick = getJoystick(port);
	if (stick == NULL) return 0;
	return stick->GetX((Joystick::JoystickHand) hand);
}

/**
 * Get the Y value of the joystick.
 * This depends on the mapping of the joystick connected to the current port.
 *
 * @param port The USB port for this joystick.
 */
float GetY(UINT32 port, JoystickHand hand)
{
	Joystick *stick = getJoystick(port);
	if (stick == NULL) return 0;
	return stick->GetY((Joystick::JoystickHand) hand);
}

/**
 * Get the Z value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 *
 * @param port The USB port for this joystick.
 */
float GetZ(UINT32 port)
{
	Joystick *stick = getJoystick(port);
	if (stick == NULL) return 0;
	return stick->GetZ();
}

/**
 * Get the twist value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 *
 * @param port The USB port for this joystick.
 */
float GetTwist(UINT32 port)
{
	Joystick *stick = getJoystick(port);
	if (stick == NULL) return 0;
	return stick->GetTwist();
}

/**
 * Get the throttle value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 *
 * @param port The USB port for this joystick.
 */
float GetThrottle(UINT32 port)
{
	Joystick *stick = getJoystick(port);
	if (stick == NULL) return 0;
	return stick->GetThrottle();
}

/**
 * For the current joystick, return the axis determined by the argument.
 *
 * This is for cases where the joystick axis is returned programatically, otherwise one of the
 * previous functions would be preferable (for example GetX()).
 *
 * @param port The USB port for this joystick.
 * @param axis The axis to read.
 * @return The value of the axis.
 */
float GetAxis(UINT32 port, AxisType axis)
{
	Joystick *stick = getJoystick(port);
	if (stick == NULL) return 0;
	return stick->GetAxis((Joystick::AxisType) axis);
}

/**
 * Get the value of the axis.
 *
 * @param port The USB port for this joystick.
 * @param axis The axis to read [1-6].
 * @return The value of the axis.
 */
float GetRawAxis(UINT32 port, UINT32 axis)
{
	Joystick *stick = getJoystick(port);
	if (stick == NULL) return 0;
	return stick->GetRawAxis(axis);
}

/**
 * Read the state of the trigger on the joystick.
 *
 * Look up which button has been assigned to the trigger and read its state.
 *
 * @param port The USB port for this joystick.
 * @param hand This parameter is ignored for the Joystick class and is only here to complete the GenericHID interface.
 * @return The state of the trigger.
 */
bool GetTrigger(UINT32 port, JoystickHand hand)
{
	Joystick *stick = getJoystick(port);
	if (stick == NULL) return 0;
	return stick->GetTrigger((Joystick::JoystickHand) hand);
}

/**
 * Read the state of the top button on the joystick.
 *
 * Look up which button has been assigned to the top and read its state.
 *
 * @param port The USB port for this joystick.
 * @param hand This parameter is ignored for the Joystick class and is only here to complete the GenericHID interface.
 * @return The state of the top button.
 */
bool GetTop(UINT32 port, JoystickHand hand)
{
	Joystick *stick = getJoystick(port);
	if (stick == NULL) return 0;
	return stick->GetTop((Joystick::JoystickHand) hand);
}

/**
 * This is not supported for the Joystick.
 * This method is only here to complete the GenericHID interface.
 *
 * @param port The USB port for this joystick.
 */
bool GetBumper(UINT32 port, JoystickHand hand)
{
	Joystick *stick = getJoystick(port);
	if (stick == NULL) return 0;
	return stick->GetBumper((Joystick::JoystickHand) hand);
}

/**
 * Get buttons based on an enumerated type.
 *
 * The button type will be looked up in the list of buttons and then read.
 *
 * @param port The USB port for this joystick.
 * @param button The type of button to read.
 * @return The state of the button.
 */
bool GetButton(UINT32 port, ButtonType button)
{
	Joystick *stick = getJoystick(port);
	if (stick == NULL) return 0;
	return stick->GetButton((Joystick::ButtonType) button);
}

/**
 * Get the button value for buttons 1 through 12.
 *
 * The buttons are returned in a single 16 bit value with one bit representing the state
 * of each button. The appropriate button is returned as a boolean value.
 *
 * @param port The USB port for this joystick.
 * @param button The button number to be read.
 * @return The state of the button.
 **/
bool GetRawButton(UINT32 port, UINT32 button)
{
	Joystick *stick = getJoystick(port);
	if (stick == NULL) return 0;
	return stick->GetRawButton(button);
}


