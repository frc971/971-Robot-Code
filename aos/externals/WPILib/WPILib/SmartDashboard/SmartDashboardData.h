/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __SMART_DASHBOARD_DATA__
#define __SMART_DASHBOARD_DATA__

#include <string>

class NetworkTable;

class SmartDashboardData
{
public:
	virtual std::string GetType() = 0;
	virtual NetworkTable *GetTable() = 0;
};

#endif
