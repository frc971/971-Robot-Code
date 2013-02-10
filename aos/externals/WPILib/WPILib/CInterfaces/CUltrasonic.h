/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/
#ifndef C_ULTRASONIC_H
#define C_ULTRASONIC_H

#include "Ultrasonic.h"

void InitUltrasonic(UINT32 pingChannel, UINT32 echoChannel);
void InitUltrasonic(UINT8 pingModuleNumber, UINT32 pingChannel, UINT8 echoModuleNumber, UINT32 echoChannel);
double GetUltrasonicInches(UINT32 pingChannel, UINT32 echoChannel);
double GetUltrasonicInches(UINT8 pingModuleNumber, UINT32 pingChannel, UINT8 echoModuleNumber, UINT32 echoChannel);
double GetUltrasonicMM(UINT32 pingChannel, UINT32 echoChannel);
double GetUltrasonicMM(UINT8 pingModuleNumber, UINT32 pingChannel, UINT8 echoModuleNumber, UINT32 echoChannel);
void DeleteUltrasonic(UINT32 pingChannel, UINT32 echoChannel);
void DeleteUltrasonic(UINT8 pingModuleNumber, UINT32 pingChannel, UINT8 echoModuleNumber, UINT32 echoChannel);

typedef void *UltrasonicObject;

UltrasonicObject CreateUltrasonic(UINT32 pingChannel, UINT32 echoChannel);
UltrasonicObject CreateUltrasonic(UINT8 pingModuleNumber, UINT32 pingChannel, UINT8 echoModuleNumber, UINT32 echoChannel);
double GetUltrasonicInches(UltrasonicObject o);
double GetUltrasonicMM(UltrasonicObject o);
void DeleteUltrasonic(UltrasonicObject o);

#endif
