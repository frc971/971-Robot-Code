/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef C_ROBOT_DRIVE_H
#define C_ROBOT_DRIVE_H

#include <VxWorks.h>

void CreateRobotDrive(UINT32 leftMotor, UINT32 rightMotor);
void CreateRobotDrive(UINT32 frontLeftMotor, UINT32 rearLeftMotor,
		UINT32 frontRightMotor, UINT32 rearRightMotor);
void Drive(float speed, float curve);
void TankDrive(UINT32 leftStickPort, UINT32 rightStickPort);
void ArcadeDrive(UINT32 stickPort, bool squaredInputs = false);
void TankByValue(float leftSpeed, float rightSpeed);
void ArcadeByValue(float moveSpeed, float rotateSpeed, bool squaredInputs = false);

#endif

