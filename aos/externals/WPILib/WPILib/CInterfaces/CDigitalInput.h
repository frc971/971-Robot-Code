/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef _C_DIGITAL_INPUT_H
#define _C_DIGITIL_INPUT_H

UINT32 GetDigitalInput(UINT8 moduleNumber, UINT32 channel);
UINT32 GetDigitalInput(UINT32 channel);
void DeleteDigitalInput(UINT8 moduleNumber, UINT32 channel);
void DeleteDigitalInput(UINT32 channel);

typedef void *DigitalInputObject;

DigitalInputObject CreateDigitalInput(UINT8 moduleNumber, UINT32 channel);
DigitalInputObject CreateDigitalInput(UINT32 channel);
bool GetDigitalInput(DigitalInputObject o);
void DeleteDigitalInput(DigitalInputObject o);

#endif

