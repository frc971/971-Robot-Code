/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef _C_SOLENOID_H
#define _C_SOLENOID_H

void SetSolenoid(UINT32 channel, bool on);
bool GetSolenoid(UINT32 channel);

typedef void *SolenoidObject;

SolenoidObject CreateSolenoid(UINT32 channel);
void DeleteSolenoid(SolenoidObject o);
void SetSolenoid(SolenoidObject o, bool on);
bool GetSolenoid(SolenoidObject o);

#endif

