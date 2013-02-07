/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __TASK_H__
#define __TASK_H__

#include "ErrorBase.h"
#include <vxWorks.h>

/**
 * WPI task is a wrapper for the native Task object.
 * All WPILib tasks are managed by a static task manager for simplified cleanup.
 **/
class Task : public ErrorBase
{
public:
	static const UINT32 kDefaultPriority = 101;
	static const INT32 kInvalidTaskID = -1;

	Task(const char* name, FUNCPTR function, INT32 priority = kDefaultPriority, UINT32 stackSize = 20000);
	virtual ~Task();

	bool Start(UINT32 arg0 = 0, UINT32 arg1 = 0, UINT32 arg2 = 0, UINT32 arg3 = 0, UINT32 arg4 = 0, 
			UINT32 arg5 = 0, UINT32 arg6 = 0, UINT32 arg7 = 0, UINT32 arg8 = 0, UINT32 arg9 = 0);
	bool Restart();
	bool Stop();

	bool IsReady();
	bool IsSuspended();

	bool Suspend();
	bool Resume();

	bool Verify();

	INT32 GetPriority();
	bool SetPriority(INT32 priority);
	const char* GetName();
	INT32 GetID();

private:
	FUNCPTR m_function;
	char* m_taskName;
	INT32 m_taskID;
	UINT32 m_stackSize;
	INT32 m_priority;
	bool HandleError(STATUS results);
	DISALLOW_COPY_AND_ASSIGN(Task);
};

#endif // __TASK_H__
