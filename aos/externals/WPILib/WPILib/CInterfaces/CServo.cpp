/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "Servo.h"
#include "CInterfaces/CServo.h"
#include "CInterfaces/CPWM.h"

static SensorBase *CreateServoStatic(UINT32 slot, UINT32 channel)
{
	return new Servo(slot, channel);
}

/**
 * Set the servo position.
 *
 * Servo values range from 0.0 to 1.0 corresponding to the range of full left to full right.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The PWM channel in the module the servo is plugged into
 * @param value Position from 0.0 to 1.0.
 */
void SetServo(UINT32 slot, UINT32 channel, float value)
{
	Servo *servo = (Servo *) AllocatePWM(slot, channel, CreateServoStatic);
	servo->Set(value);
}

/**
 * Get the servo position.
 *
 * Servo values range from 0.0 to 1.0 corresponding to the range of full left to full right.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The PWM channel in the module the servo is plugged into
 * @return Position from 0.0 to 1.0.
 */
float GetGetServo(UINT32 slot, UINT32 channel)
{
	Servo *servo = (Servo *) AllocatePWM(slot, channel, CreateServoStatic);
	return servo->Get();
}

/**
 * Set the servo angle.
 *
 * Assume that the servo angle is linear with respect to the PWM value (big assumption, need to test).
 *
 * Servo angles that are out of the supported range of the servo simply "saturate" in that direction
 * In other words, if the servo has a range of (X degrees to Y degrees) than angles of less than X
 * result in an angle of X being set and angles of more than Y degrees result in an angle of Y being set.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The PWM channel in the module the servo is plugged into
 * @param angle The angle in degrees to set the servo.
 */
void SetServoAngle(UINT32 slot, UINT32 channel, float angle)
{
	Servo *servo = (Servo *) AllocatePWM(slot, channel, CreateServoStatic);
	servo->SetAngle(angle);
}

/**
 * Get the servo angle.
 *
 * Assume that the servo angle is linear with respect to the PWM value (big assumption, need to test).
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The PWM channel in the module the servo is plugged into
 * @return The angle in degrees to which the servo is set.
 */
float GetServoAngle(UINT32 slot, UINT32 channel)
{
	Servo *servo = (Servo *) AllocatePWM(slot, channel, CreateServoStatic);
	return servo->GetAngle();
}

/**
 * Get the maximum servo angle.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The PWM port in the module the servo is plugged into
 */
float GetServoMaxAngle(UINT32 slot, UINT32 channel)
{
	Servo *servo = (Servo *) AllocatePWM(slot, channel, CreateServoStatic);
	return servo->GetMaxAngle();
}

/**
 * Get the minimum servo angle.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The PWM port in the module the servo is plugged into
 */
float GetServoMinAngle(UINT32 slot, UINT32 channel)
{
	Servo *servo = (Servo *) AllocatePWM(slot, channel, CreateServoStatic);
	return servo->GetMinAngle();
}

/**
 * Set the servo position.
 *
 * Servo values range from 0.0 to 1.0 corresponding to the range of full left to full right.
 *
 * @param channel The PWM port in the module the servo is plugged into
 * @param value Position from 0.0 to 1.0.
 */
void SetServo(UINT32 channel, float value)
{
	Servo *servo = (Servo *) AllocatePWM(channel, CreateServoStatic);
	servo->Set(value);
}

/**
 * Get the servo position.
 *
 * Servo values range from 0.0 to 1.0 corresponding to the range of full left to full right.
 *
 * @param channel The PWM port in the module the servo is plugged into
 * @returns Position from 0.0 to 1.0.
 */
float GetServo(UINT32 channel)
{
	Servo *servo = (Servo *) AllocatePWM(channel, CreateServoStatic);
	return servo->Get();
}

/**
 * Set the servo angle.
 *
 * Assume that the servo angle is linear with respect to the PWM value (big assumption, need to test).
 *
 * Servo angles that are out of the supported range of the servo simply "saturate" in that direction
 * In other words, if the servo has a range of (X degrees to Y degrees) than angles of less than X
 * result in an angle of X being set and angles of more than Y degrees result in an angle of Y being set.
 *
 * @param channel The PWM port in the module the servo is plugged into
 * @param angle The angle in degrees to set the servo.
 */
void SetServoAngle(UINT32 channel, float angle)
{
	Servo *servo = (Servo *) AllocatePWM(channel, CreateServoStatic);
	servo->SetAngle(angle);
}

/**
 * Get the servo angle.
 *
 * Assume that the servo angle is linear with respect to the PWM value (big assumption, need to test).
 *
 * @param channel The slot the digital module is plugged into
 * @return The angle in degrees to which the servo is set.
 */
float GetServoAngle(UINT32 channel)
{
	Servo *servo = (Servo *) AllocatePWM(channel, CreateServoStatic);
	return servo->GetAngle();
}

/**
 * Get the maximum angle for the servo.
 *
 * @param channel The PWM port in the module the servo is plugged into
 */
float GetServoMaxAngle(UINT32 channel)
{
	Servo *servo = (Servo *) AllocatePWM(channel, CreateServoStatic);
	return servo->GetMaxAngle();
}

/**
 * Get the minimum angle for the servo.
 *
 * @param channel The PWM port in the module the servo is plugged into
 */
float GetServoMinAngle(UINT32 channel)
{
	Servo *servo = (Servo *) AllocatePWM(channel, CreateServoStatic);
	return servo->GetMinAngle();
}

/**
 * Free the resources associated with this Servo object.
 * The underlying Servo object and the allocated ports are freed.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The PWM port in the module the servo is plugged into
 */
void DeleteServo(UINT32 slot, UINT32 channel)
{
	Servo *servo = (Servo *) AllocatePWM(slot, channel, CreateServoStatic);
	DeletePWM(slot, channel);
	delete servo;
}

/**
 * Free the resources associated with this Servo object.
 * The underlying Servo object and the allocated ports are freed.
 *
 * @param channel The PWM port in the module the servo is plugged into
 */
void DeleteServo(UINT32 channel)
{
	DeleteServo(SensorBase::GetDefaultDigitalModule(), channel);
}

ServoObject CreateServo(UINT32 slot, UINT32 channel)
{
	return (ServoObject) new Servo(slot, channel);
}

ServoObject CreateServo(UINT32 channel)
{
	return (ServoObject) new Servo(channel);
}

void SetServo(ServoObject o, float value)
{
	((Servo *)o)->Set(value);
}

float GetGetServo(ServoObject o)
{
	return ((Servo *)o)->Get();
}

void SetServoAngle(ServoObject o, float angle)
{
	((Servo *)o)->SetAngle(angle);
}

float GetServoAngle(ServoObject o)
{
	return ((Servo *)o)->GetAngle();
}

float GetServoMaxAngle(ServoObject o)
{
	return ((Servo *)o)->GetMaxAngle();
}

float GetServoMinAngle(ServoObject o)
{
	return ((Servo *)o)->GetMinAngle();
}

void DeleteServo(ServoObject o)
{
	delete (Servo *)o;
}


