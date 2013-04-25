/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef SIMPLE_ROBOT_H
#define SIMPLE_ROBOT_H

#include "RobotBase.h"

/**
 * @todo If this is going to last until release, it needs a better name.
 * Overridden Autonomous() and OperatorControl() methods are called at the appropriate time
 * as the match proceeds. In the current implementation, the Autonomous code will run to
 * completion before the OperatorControl code could start. In the future the Autonomous code
 * might be spawned as a task, then killed at the end of the Autonomous period.
 */
class SimpleRobot: public RobotBase
{
public:
	SimpleRobot();
	virtual ~SimpleRobot() {}
	virtual void RobotInit();
	virtual void Disabled();
	virtual void Autonomous();
    virtual void OperatorControl();
    virtual void Test();
	virtual void RobotMain();
	void StartCompetition();

private:
	bool m_robotMainOverridden;
};

#endif
