/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "CInterfaces/CRobotDrive.h"
#include "Joystick.h"
#include "RobotDrive.h"
#include "Utility.h"
#include "WPIErrors.h"

static RobotDrive *drive = NULL;

/*
 * Driving functions
 * These functions provide an interface to multiple motors that is used for C programming
 * The Drive(speed, direction) function is the main part of the set that makes it easy
 * to set speeds and direction independently in one call.
 */

/**
 * Create a RobotDrive with 4 motors specified with channel numbers.
 * Set up parameters for a four wheel drive system where all four motor
 * pwm channels are specified in the call.
 * This call assumes Jaguars for controlling the motors.
 *
 * @param frontLeftMotor Front left motor channel number on the default digital module
 * @param rearLeftMotor Rear Left motor channel number on the default digital module
 * @param frontRightMotor Front right motor channel number on the default digital module
 * @param rearRightMotor Rear Right motor channel number on the default digital module
 */
void CreateRobotDrive(UINT32 frontLeftMotor, UINT32 rearLeftMotor,
		UINT32 frontRightMotor, UINT32 rearRightMotor)
{
	if (drive == NULL)
		drive = new RobotDrive(frontLeftMotor, rearLeftMotor,
								frontRightMotor, rearRightMotor);
}

/**
 * Constructor for RobotDrive with 2 motors specified with channel numbers.
 * Set up parameters for a four wheel drive system where all four motor
 * pwm channels are specified in the call.
 * This call assumes Jaguars for controlling the motors.
 *
 * @param leftMotor Front left motor channel number on the default digital module
 * @param rightMotor Front right motor channel number on the default digital module
 */
void CreateRobotDrive(UINT32 leftMotor, UINT32 rightMotor)
{
	if (drive == NULL)
		drive = new RobotDrive(leftMotor, rightMotor);
}

/**
 * Drive the motors at "speed" and "curve".
 *
 * The speed and curve are -1.0 to +1.0 values where 0.0 represents stopped and
 * not turning. The algorithm for adding in the direction attempts to provide a constant
 * turn radius for differing speeds.
 *
 * This function sill most likely be used in an autonomous routine.
 *
 * @param speed The forward component of the speed to send to the motors.
 * @param curve The rate of turn, constant for different forward speeds.
 */
void Drive(float speed, float curve)
{
	if (drive == NULL)
		wpi_setGlobalWPIError(DriveUninitialized);
	else
		drive->Drive(speed, curve);
}

/**
 * Provide tank steering using the stored robot configuration.
 * Drive the robot using two joystick inputs. The Y-axis will be selected from
 * each Joystick object.
 *
 * @param leftStickPort The joystick port to control the left side of the robot.
 * @param rightStickPort The joystick port to control the right side of the robot.
 */
void TankDrive(UINT32 leftStickPort, UINT32 rightStickPort)
{
	if (drive == NULL)
	{
		wpi_setGlobalWPIError(DriveUninitialized);
	}
	else
	{
		Joystick *leftStick = Joystick::GetStickForPort(leftStickPort);
		Joystick *rightStick = Joystick::GetStickForPort(rightStickPort);
		drive->TankDrive(leftStick, rightStick);
	}
}

/**
 * Arcade drive implements single stick driving.
 * Given a single Joystick, the class assumes the Y axis for the move value and the X axis
 * for the rotate value.
 * (Should add more information here regarding the way that arcade drive works.)
 *
 * @param stickPort The joystick to use for Arcade single-stick driving. The Y-axis will be selected
 * for forwards/backwards and the X-axis will be selected for rotation rate.
 * @param squaredInputs If true, the sensitivity will be increased for small values
 */
void ArcadeDrive(UINT32 stickPort, bool squaredInputs)
{
	if (drive == NULL)
	{
		wpi_setGlobalWPIError(DriveUninitialized);
	}
	else
	{
		Joystick *stick = Joystick::GetStickForPort(stickPort);
		drive->ArcadeDrive(stick);
	}
}

/**
 * Provide tank steering using the stored robot configuration.
 * This function lets you directly provide joystick values from any source.
 * @param leftSpeed The value of the left stick.
 * @param rightSpeed The value of the right stick.
 */
void TankByValue(float leftSpeed, float rightSpeed)
{
	if (drive == NULL)
	{
		wpi_setGlobalWPIError(DriveUninitialized);
	}
	else
	{
		drive->Drive(leftSpeed, rightSpeed);
	}
}

/**
 * Arcade drive implements single stick driving.
 * This function lets you directly provide joystick values from any source.
 *
 * @param moveValue The value to use for fowards/backwards
 * @param rotateValue The value to use for the rotate right/left
 * @param squaredInputs If set, increases the sensitivity at low speeds
 */
void ArcadeByValue(float moveValue, float rotateValue, bool squaredInputs)
{
	if (drive == NULL)
		wpi_setGlobalWPIError(DriveUninitialized);
	else
		drive->ArcadeDrive(moveValue, rotateValue, squaredInputs);
}
