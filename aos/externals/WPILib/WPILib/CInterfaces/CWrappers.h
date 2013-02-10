/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef C_WRAPPERS_H
#define C_WRAPPERS_H

class SensorBase;
typedef SensorBase *(*SensorCreator)(UINT32 slot, UINT32 channel);

#endif

