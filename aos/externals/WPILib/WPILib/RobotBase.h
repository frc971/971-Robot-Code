/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef ROBOT_H_
#define ROBOT_H_

#include "Base.h"
#include "Task.h"
#include "Watchdog.h"

class DriverStation;

/**
 * This macro will set up the given class (which must be a (direct or indirect)
 * RobotBase subclass) so that when the user code is loaded, it will be
 * instantiated and StartCompetition() will be called on the instance.
 */
#define START_ROBOT_CLASS(_ClassName_) \
	RobotBase *FRC_userClassFactory() \
	{ \
		return new _ClassName_(); \
	} \
	extern "C" { \
		INT32 FRC_UserProgram_StartupLibraryInit() \
		{ \
			RobotBase::startRobotTask((FUNCPTR)FRC_userClassFactory); \
			return 0; \
		} \
	}

/**
 * Implement a Robot Program framework.
 * The RobotBase class is intended to be subclassed by a user creating a robot
 * program, possibly indirectly through one of the subclasses included in this
 * library.
 */
class RobotBase {
	friend class RobotDeleter;
public:
	static RobotBase &getInstance();
	static void setInstance(RobotBase* robot);

	bool IsEnabled();
	bool IsDisabled();
	bool IsAutonomous();
	bool IsOperatorControl();
    bool IsTest();
	bool IsSystemActive();
	bool IsNewDataAvailable();
	Watchdog &GetWatchdog();
	static void startRobotTask(FUNCPTR factory);
	static void robotTask(FUNCPTR factory, Task *task);

protected:
	virtual ~RobotBase();
	virtual void StartCompetition() = 0;
	RobotBase();

	Task *m_task;
	Watchdog m_watchdog;
	DriverStation *m_ds;

private:
	static RobotBase *m_instance;
	DISALLOW_COPY_AND_ASSIGN(RobotBase);
};

#endif

