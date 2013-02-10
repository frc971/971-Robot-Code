/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __SUBSYSTEM_H__
#define __SUBSYSTEM_H__

#include "ErrorBase.h"
#include "SmartDashboard/SmartDashboardNamedData.h"
#include <string>

class NetworkTable;
class Command;

class Subsystem : public SmartDashboardNamedData, public ErrorBase
{
	friend class Scheduler;
public:
	Subsystem(const char *name);
	virtual ~Subsystem() {}

	virtual std::string GetName();
	virtual std::string GetType();
	virtual NetworkTable *GetTable();

	void SetDefaultCommand(Command *command);
	Command *GetDefaultCommand();
	void SetCurrentCommand(Command *command);
	Command *GetCurrentCommand();
	virtual void InitDefaultCommand();
	
private:
	void ConfirmCommand();

	NetworkTable *m_table;
	Command *m_currentCommand;
	Command *m_defaultCommand;
	std::string m_name;
	bool m_initializedDefaultCommand;
};

#endif
