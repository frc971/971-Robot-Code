/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef C_ANALOG_CHANNEL_H
#define C_ANALOG_CHANNEL_H

#include "AnalogChannel.h"
#include "CWrappers.h"

AnalogChannel *AllocateAnalogChannel(UINT8 moduleNumber, UINT32 channel /*,SensorCreator createObject*/);

INT16 GetAnalogValue(UINT8 moduleNumber, UINT32 channel);
INT32 GetAnalogAverageValue(UINT8 moduleNumber, UINT32 channel);

float GetAnalogVoltage(UINT8 moduleNumber, UINT32 channel);
float GetAnalogAverageVoltage(UINT8 moduleNumber, UINT32 channel);

void SetAnalogAverageBits(UINT8 moduleNumber, UINT32 channel, UINT32 bits);
UINT32 GetAnalogAverageBits(UINT8 moduleNumber, UINT32 channel);
void SetAnalogOversampleBits(UINT8 moduleNumber, UINT32 channel, UINT32 bits);
UINT32 GetAnalogOversampleBits(UINT8 moduleNumber, UINT32 channel);

INT16 GetAnalogValue(UINT32 channel);
INT32 GetAnalogAverageValue(UINT32 channel);

float GetAnalogVoltage(UINT32 channel);
float GetAnalogAverageVoltage(UINT32 channel);

void SetAnalogAverageBits(UINT32 channel, UINT32 bits);
UINT32 GetAnalogAverageBits(UINT32 channel);
void SetAnalogOversampleBits(UINT32 channel, UINT32 bits);
UINT32 GetAnalogOversampleBits(UINT32 channel);

UINT32 GetAnalogLSBWeight();
INT32 GetAnalogOffset();

void DeleteAnalogChannel(UINT8 moduleNumber, UINT32 channel);
void DeleteAnalogChannel(UINT32 channel);

typedef void *AnalogChannelObject;

AnalogChannelObject CreateAnalogChannel(UINT8 moduleNumber, UINT32 channel);
AnalogChannelObject CreateAnalogChannel(UINT32 channel);
INT16 GetAnalogValue(AnalogChannelObject o);
INT32 GetAnalogAverageValue(AnalogChannelObject o);

float GetAnalogVoltage(AnalogChannelObject o);
float GetAnalogAverageVoltage(AnalogChannelObject o);

void SetAnalogAverageBits(AnalogChannelObject o, UINT32 bits);
UINT32 GetAnalogAverageBits(AnalogChannelObject o);
void SetAnalogOversampleBits(AnalogChannelObject o, UINT32 bits);
UINT32 GetAnalogOversampleBits(AnalogChannelObject o);
void DeleteAnalogChannel(AnalogChannelObject o);

#endif

