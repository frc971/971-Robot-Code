/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "SimpleRobot.h"

#include "DriverStation.h"
#include "NetworkCommunication/UsageReporting.h"
#include "Timer.h"

SimpleRobot::SimpleRobot()
	: m_robotMainOverridden (true)
{
	m_watchdog.SetEnabled(false);
}

/**
 * Robot-wide initialization code should go here.
 * 
 * Programmers should override this method for default Robot-wide initialization which will
 * be called when the robot is first powered on.  It will be called exactly 1 time.
 */
void SimpleRobot::RobotInit()
{
	printf("Default %s() method... Override me!\n", __FUNCTION__);
}

/**
 * Disabled should go here.
 * Programmers should override this method to run code that should run while the field is
 * disabled.
 */
void SimpleRobot::Disabled()
{
	printf("Default %s() method... Override me!\n", __FUNCTION__);
}

/**
 * Autonomous should go here.
 * Programmers should override this method to run code that should run while the field is
 * in the autonomous period.
 */
void SimpleRobot::Autonomous()
{
	printf("Default %s() method... Override me!\n", __FUNCTION__);
}

/**
 * Operator control (tele-operated) code should go here.
 * Programmers should override this method to run code that should run while the field is
 * in the Operator Control (tele-operated) period.
 */
void SimpleRobot::OperatorControl()
{
	printf("Default %s() method... Override me!\n", __FUNCTION__);
}

/**
 * Robot main program for free-form programs.
 * 
 * This should be overridden by user subclasses if the intent is to not use the Autonomous() and
 * OperatorControl() methods. In that case, the program is responsible for sensing when to run
 * the autonomous and operator control functions in their program.
 * 
 * This method will be called immediately after the constructor is called. If it has not been
 * overridden by a user subclass (i.e. the default version runs), then the Autonomous() and
 * OperatorControl() methods will be called.
 */
void SimpleRobot::RobotMain()
{
	m_robotMainOverridden = false;
}

/**
 * Start a competition.
 * This code needs to track the order of the field starting to ensure that everything happens
 * in the right order. Repeatedly run the correct method, either Autonomous or OperatorControl
 * when the robot is enabled. After running the correct method, wait for some state to change,
 * either the other mode starts or the robot is disabled. Then go back and wait for the robot
 * to be enabled again.
 */
void SimpleRobot::StartCompetition()
{
	nUsageReporting::report(nUsageReporting::kResourceType_Framework, nUsageReporting::kFramework_Simple);

	RobotMain();
	
	if (!m_robotMainOverridden)
	{
		// first and one-time initialization
		RobotInit();

		while (true)
		{
			if (IsDisabled())
			{
				m_ds->InDisabled(true);
				Disabled();
				m_ds->InDisabled(false);
				while (IsDisabled()) m_ds->WaitForData();
			}
			else if (IsAutonomous())
			{
				m_ds->InAutonomous(true);
				Autonomous();
				m_ds->InAutonomous(false);
				while (IsAutonomous() && IsEnabled()) m_ds->WaitForData();
			}
			else
			{
				m_ds->InOperatorControl(true);
				OperatorControl();
				m_ds->InOperatorControl(false);
				while (IsOperatorControl() && IsEnabled()) m_ds->WaitForData();
			}
		}
	}
}
