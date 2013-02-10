/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef C_JAGUAR_H
#define C_JAGUAR_H

void SetJaguarSpeed(UINT32 module, UINT32 channel, float speed);
void SetJaguarSpeed(UINT32 channel, float speed);
void SetJaguarRaw(UINT32 channel, UINT8 value);
UINT8 GetJaguarRaw(UINT32 channel);
void SetJaguarRaw(UINT32 module, UINT32 channel, UINT8 value);
UINT8 GetJaguarRaw(UINT32 module, UINT32 channel);
void DeleteJaguar(UINT32 module, UINT32 channel);
void DeleteJaguar(UINT32 channel);

typedef void *JaguarObject;

JaguarObject CreateJaguar(UINT32 module, UINT32 channel);
JaguarObject CreateJaguar(UINT32 channel);
void SetJaguarRaw(JaguarObject o, UINT8 value);
void SetJaguarSpeed(JaguarObject o, float speed);
UINT8 GetJaguarRaw(JaguarObject o);
void DeleteJaguar(JaguarObject o);

#endif

