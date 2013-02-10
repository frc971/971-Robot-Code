/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef _C_DIGITAL_OUTPUT_H
#define _C_DIGITIL_OUTPUT_H

void SetDigitalOutput(UINT8 moduleNumber, UINT32 channel, UINT32 value);
void SetDigitalOutput(UINT32 channel, UINT32 value);
void DeleteDigitalOutput(UINT8 moduleNumber, UINT32 channel);
void DeleteDigitalOutput(UINT32 channel);

typedef void *DigitalOutputObject;

DigitalOutputObject CreateDigitalOutput(UINT8 moduleNumber, UINT32 channel);
DigitalOutputObject CreateDigitalOutput(UINT32 channel);
void SetDigitalOutput(DigitalOutputObject o, bool val);
void DeleteDigitalOutput(DigitalOutputObject o);

#endif

