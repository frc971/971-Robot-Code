/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include "Commands/Command.h"
#include "ErrorBase.h"
#include "SmartDashboard/SmartDashboardNamedData.h"
#include <list>
#include <map>
#include <set>
#include <vector>

class ButtonScheduler;
class NetworkTable;
class Subsystem;

class Scheduler : public SmartDashboardNamedData, public ErrorBase
{
public:
	static Scheduler *GetInstance();

	virtual std::string GetName();
	virtual std::string GetType();
	virtual NetworkTable *GetTable();

	void AddCommand(Command* command);
	void AddButton(ButtonScheduler* button);
	void RegisterSubsystem(Subsystem *subsystem);
	void Run();	
	void Remove(Command *command);

private:
	Scheduler();
	virtual ~Scheduler();

	void ProcessCommandAddition(Command *command);

	static Scheduler *_instance;
	SEM_ID m_tableLock;
	NetworkTable *m_table;
	Command::SubsystemSet m_subsystems;
	SEM_ID m_buttonsLock;
	typedef std::vector<ButtonScheduler *> ButtonVector;
	ButtonVector m_buttons;
	typedef std::vector<Command *> CommandVector;
	SEM_ID m_additionsLock;
	CommandVector m_additions;
	typedef std::set<Command *> CommandSet;
	CommandSet m_commands;
	bool m_adding;
};
#endif

