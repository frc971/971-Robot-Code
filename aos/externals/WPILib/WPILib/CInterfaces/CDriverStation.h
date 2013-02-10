/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef _C_DRIVER_STATION_H
#define _C_DRIVER_STATION_H

float GetStickAxis(UINT32 stick, UINT32 axis);
short GetStickButtons(UINT32 stick);

float GetAnalogIn(UINT32 channel);
bool GetDigitalIn(UINT32 channel);
void SetDigitalOut(UINT32 channel, bool value);
bool GetDigitalOut(UINT32 channel);

bool IsDisabled();
bool IsAutonomous();
bool IsOperatorControl();

UINT32 GetPacketNumber();
UINT32 GetAlliance();
UINT32 GetLocation();

float GetBatteryVoltage();

#endif
