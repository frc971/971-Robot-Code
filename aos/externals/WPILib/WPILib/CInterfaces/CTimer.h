/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/
#ifndef _C_TIMER_H
#define _C_TIMER_H

#include "Timer.h"

static const unsigned kMaxTimers = 32;

void ResetTimer(UINT32 index);
void StartTimer(UINT32 index);
void StopTimer(UINT32 index);
double GetTimer(UINT32 index);
void DeleteTimer(UINT32 index);

#endif

