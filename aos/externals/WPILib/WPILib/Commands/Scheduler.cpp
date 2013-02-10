/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "Commands/Scheduler.h"

#include "Buttons/ButtonScheduler.h"
#include "Commands/Subsystem.h"
#include "NetworkCommunication/UsageReporting.h"
#include "NetworkTables/NetworkTable.h"
#include "Synchronized.h"
#include "WPIErrors.h"
#include <iostream>
#include <set>
#include <semLib.h>

Scheduler *Scheduler::_instance = NULL;

Scheduler::Scheduler() :
	m_tableLock(NULL),
	m_table(NULL),
	m_buttonsLock(NULL),
	m_additionsLock(NULL),
	m_adding(false)
{
	m_tableLock = semMCreate(SEM_Q_PRIORITY | SEM_INVERSION_SAFE | SEM_DELETE_SAFE);
	m_buttonsLock = semMCreate(SEM_Q_PRIORITY | SEM_INVERSION_SAFE | SEM_DELETE_SAFE);
	m_additionsLock = semMCreate(SEM_Q_PRIORITY | SEM_INVERSION_SAFE | SEM_DELETE_SAFE);

	nUsageReporting::report(nUsageReporting::kResourceType_Command, nUsageReporting::kCommand_Scheduler);
}

Scheduler::~Scheduler()
{
	semTake(m_additionsLock, WAIT_FOREVER);
	semDelete(m_additionsLock);

	semTake(m_buttonsLock, WAIT_FOREVER);
	semDelete(m_buttonsLock);

	semTake(m_tableLock, WAIT_FOREVER);
	semDelete(m_tableLock);
}

/**
 * Returns the {@link Scheduler}, creating it if one does not exist.
 * @return the {@link Scheduler}
 */
Scheduler *Scheduler::GetInstance()
{
	if (_instance == NULL)
		_instance = new Scheduler();
	return _instance;
}

void Scheduler::AddCommand(Command *command)
{
	Synchronized sync(m_additionsLock);
	m_additions.push_back(command);
}

void Scheduler::AddButton(ButtonScheduler *button)
{
	Synchronized sync(m_buttonsLock);
	m_buttons.push_back(button);
}

void Scheduler::ProcessCommandAddition(Command *command)
{
	if (command == NULL)
		return;

	// Check to make sure no adding during adding
	if (m_adding)
	{
		wpi_setWPIErrorWithContext(IncompatibleState, "Can not start command from cancel method");
		return;
	}

	// Only add if not already in
	CommandSet::iterator found = m_commands.find(command);
	if (found == m_commands.end())
	{
		// Check that the requirements can be had
		Command::SubsystemSet requirements = command->GetRequirements();
		Command::SubsystemSet::iterator iter;
		for (iter = requirements.begin(); iter != requirements.end(); iter++)
		{
			Subsystem *lock = *iter;
			if (lock->GetCurrentCommand() != NULL && !lock->GetCurrentCommand()->IsInterruptible())
				return;
		}

		// Give it the requirements
		m_adding = true;
		for (iter = requirements.begin(); iter != requirements.end(); iter++)
		{
			Subsystem *lock = *iter;
			if (lock->GetCurrentCommand() != NULL)
			{
				lock->GetCurrentCommand()->Cancel();
				Remove(lock->GetCurrentCommand());
			}
			lock->SetCurrentCommand(command);
		}
		m_adding = false;

		m_commands.insert(command);

		command->StartRunning();
	}
}

/**
 * Runs a single iteration of the loop.  This method should be called often in order to have a functioning
 * {@link Command} system.  The loop has five stages:
 *
 * <ol>
 * <li> Poll the Buttons </li>
 * <li> Execute/Remove the Commands </li>
 * <li> Send values to SmartDashboard </li>
 * <li> Add Commands </li>
 * <li> Add Defaults </li>
 * </ol>
 */
void Scheduler::Run()
{
	// Get button input (going backwards preserves button priority)
	{
		Synchronized sync(m_buttonsLock);
		ButtonVector::reverse_iterator rButtonIter = m_buttons.rbegin();
		for (; rButtonIter != m_buttons.rend(); rButtonIter++)
		{
			(*rButtonIter)->Execute();
		}
	}

	// Loop through the commands
	CommandSet::iterator commandIter = m_commands.begin();
	for (; commandIter != m_commands.end();)
	{
		Command *command = *commandIter;
		// Increment before potentially removing to keep the iterator valid
		commandIter++;
		if (!command->Run())
		{
			Remove(command);
		}
	}

	// Send the value over the table
	if (m_table != NULL) {
		int count = 0;
		Synchronized sync(m_tableLock);
		m_table->BeginTransaction();
		commandIter = m_commands.begin();
		for (; commandIter != m_commands.end(); commandIter++)
		{
			char buf[10];
			snprintf(buf, 10, "%d", ++count);
			m_table->PutSubTable(buf, (*commandIter)->GetTable());
		}
		m_table->PutInt("count", count);
		m_table->EndTransaction();
	}

	// Add the new things
	{
		Synchronized sync(m_additionsLock);
		CommandVector::iterator additionsIter = m_additions.begin();
		for (; additionsIter != m_additions.end(); additionsIter++)
		{
			ProcessCommandAddition(*additionsIter);
		}
		m_additions.clear();
	}

	// Add in the defaults
	Command::SubsystemSet::iterator subsystemIter = m_subsystems.begin();
	for (; subsystemIter != m_subsystems.end(); subsystemIter++)
	{
		Subsystem *lock = *subsystemIter;
		if (lock->GetCurrentCommand() == NULL)
		{
			ProcessCommandAddition(lock->GetDefaultCommand());
		}
		lock->ConfirmCommand();
	}
}

/**
 * Registers a {@link Subsystem} to this {@link Scheduler}, so that the {@link Scheduler} might know
 * if a default {@link Command} needs to be run.  All {@link Subsystem Subsystems} should call this.
 * @param system the system
 */
void Scheduler::RegisterSubsystem(Subsystem *subsystem)
{
	if (subsystem == NULL)
	{
		wpi_setWPIErrorWithContext(NullParameter, "subsystem");
		return;
	}
	m_subsystems.insert(subsystem);
}

/**
 * Removes the {@link Command} from the {@link Scheduler}.
 * @param command the command to remove
 */
void Scheduler::Remove(Command *command)
{
	if (command == NULL)
	{
		wpi_setWPIErrorWithContext(NullParameter, "command");
		return;
	}

	if (!m_commands.erase(command))
		return;

	Command::SubsystemSet requirements = command->GetRequirements();
	Command::SubsystemSet::iterator iter = requirements.begin();
	for (; iter != requirements.end(); iter++)
	{
		Subsystem *lock = *iter;
		lock->SetCurrentCommand(NULL);
	}

	command->Removed();
}

std::string Scheduler::GetName()
{
    return "Scheduler";
}

std::string Scheduler::GetType()
{
    return "Scheduler";
}

NetworkTable *Scheduler::GetTable()
{
	if (m_table == NULL)
	{
		m_table = new NetworkTable();
		m_table->PutInt("count", 0);
	}
	return m_table;
}
