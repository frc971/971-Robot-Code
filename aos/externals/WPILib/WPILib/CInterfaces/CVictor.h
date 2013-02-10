/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef C_VICTOR_H
#define C_VICTOR_H

#include <VxWorks.h>

void SetVictorSpeed(UINT32 module, UINT32 channel, float speed);
void SetVictorSpeed(UINT32 channel, float speed);
void SetVictorRaw(UINT32 channel, UINT8 value);
UINT8 GetVictorRaw(UINT32 channel);
void SetVictorRaw(UINT32 module, UINT32 channel, UINT8 value);
UINT8 GetVictorRaw(UINT32 module, UINT32 channel);
void DeleteVictor(UINT32 module, UINT32 channel);
void DeleteVictor(UINT32 channel);

typedef void *VictorObject;

VictorObject CreateVictor(UINT32 slot, UINT32 channel);
VictorObject CreateVictor(UINT32 channel);
void DeleteVictor(VictorObject o);
void SetVictorSpeed(VictorObject o, float speed);
void SetVictorRaw(VictorObject o, UINT8 value);
UINT8 GetVictorRaw(VictorObject o);

void LoadVictor();
#endif
