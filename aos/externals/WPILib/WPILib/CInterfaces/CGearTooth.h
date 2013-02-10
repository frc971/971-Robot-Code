/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/
#ifndef C_GEARTOOTH_H
#define C_GEARTOOTH_H

#include "GearTooth.h"

// TODO: Need to add support for digital sensors

void InitGearTooth(UINT32 channel, bool directionSensitive);
void InitGearTooth(UINT8 moduleNumber, UINT32 channel, bool directionSensitive);
void StartGearTooth(UINT32 channel);
void StartGearTooth(UINT8 moduleNumber, UINT32 channel);
void StopGearTooth(UINT32 channel);
void StopGearTooth(UINT8 moduleNumber, UINT32 channel);
INT32 GetGearTooth(UINT32 channel);
INT32 GetGearTooth(UINT8 moduleNumber, UINT32 channel);
void ResetGearTooth(UINT32 channel);
void ResetGearTooth(UINT8 moduleNumber, UINT32 channel);
void DeleteGearTooth(UINT32 channel);
void DeleteGearTooth(UINT8 moduleNumber, UINT32 channel);

typedef void *GearToothObject;

GearToothObject CreateGearTooth(UINT32 channel, bool directionSensitive = true);
GearToothObject CreateGearTooth(UINT8 moduleNumber, UINT32 channel, bool directionSensitive = true);
void StartGearTooth(GearToothObject o);
void StopGearTooth(GearToothObject o);
INT32 GetGearTooth(GearToothObject o);
void ResetGearTooth(GearToothObject o);
void DeleteGearTooth(GearToothObject o);

#endif
