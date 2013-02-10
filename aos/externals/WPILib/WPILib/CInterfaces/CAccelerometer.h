/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef _C_ACCELEROMETER_H
#define _C_ACCELEROMETER_H

float GetAcceleration(UINT32 channel);
float GetAcceleration(UINT8 moduleNumber, UINT32 channel);
void SetAccelerometerSensitivity(UINT32 channel, float sensitivity);
void SetAccelerometerSensitivity(UINT8 moduleNumber, UINT32 channel, float sensitivity);
void SetAccelerometerZero(UINT32 channel, float zero);
void SetAccelerometerZero(UINT8 moduleNumber, UINT32 channel, float zero);
void DeleteAccelerometer(UINT32 channel);
void DeleteAccelerometer(UINT8 moduleNumber, UINT32 channel);

typedef void *AccelerometerObject;

AccelerometerObject CreateAccelerometer(UINT32 channel);
AccelerometerObject CreateAccelerometer(UINT8 moduleNumber, UINT32 channel);
float GetAcceleration(AccelerometerObject o);
void SetAccelerometerSensitivity(AccelerometerObject o, float sensitivity);
void SetAccelerometerZero(AccelerometerObject o, float zero);
void DeleteAccelerometer(AccelerometerObject o);

#endif

