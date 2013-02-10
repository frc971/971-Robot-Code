/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef SIMPLE_C_ROBOT_H
#define SIMPLE_C_ROBOT_H

#include "RobotBase.h"

void Autonomous();
void OperatorControl();
void Initialize();

bool IsAutonomous();
bool IsOperatorControl();
bool IsDisabled();

void SetWatchdogEnabled(bool enable);
void SetWatchdogExpiration(float time);
void WatchdogFeed();

class SimpleCRobot: public RobotBase
{
public:
	SimpleCRobot();
	virtual ~SimpleCRobot() {}
	void StartCompetition();
};

#endif
