/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __SMART_DASHBOARD_NAMED_DATA__
#define __SMART_DASHBOARD_NAMED_DATA__

#include "SmartDashboard/SmartDashboardData.h"

class SmartDashboardNamedData : public SmartDashboardData
{
public:
	virtual std::string GetName() = 0;
};

#endif
