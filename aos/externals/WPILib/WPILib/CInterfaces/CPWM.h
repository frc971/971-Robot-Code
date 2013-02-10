/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef C_PWM_H
#define C_PWM_H

#include <VxWorks.h>
#include "CWrappers.h"
#include "PWM.h"

PWM *AllocatePWM(UINT8 moduleNumber, UINT32 channel, SensorCreator creator);
PWM *AllocatePWM(UINT32 channel, SensorCreator creator);
void DeletePWM(UINT8 moduleNumber, UINT32 channel);

#endif
