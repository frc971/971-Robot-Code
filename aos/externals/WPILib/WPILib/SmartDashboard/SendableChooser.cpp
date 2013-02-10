/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "SmartDashboard/SendableChooser.h"

#include "NetworkTables/NetworkTable.h"
#include <stdio.h>

static const char *kDefault = "default";
static const char *kCount = "count";
static const char *kSelected = "selected";

SendableChooser::SendableChooser()
{
	m_table = new NetworkTable();
	m_count = 0;
}

/**
 * Adds the given object to the list of options.  On the {@link SmartDashboard} on the desktop,
 * the object will appear as the given name.
 * @param name the name of the option
 * @param object the option
 */
void SendableChooser::AddObject(const char *name, void *object)
{
	std::pair<std::map<std::string, void *>::iterator, bool> ret = m_choices.insert(std::pair<std::string, void *>(name, object));
	if (ret.second)
	{
		//idBuf is: 10 bytes for m_count and 1 for NULL term
		char idBuf[11];
		snprintf(idBuf, 11, "%d", m_count);
		m_ids.insert(std::pair<void *, std::string>(object, idBuf));
		m_table->PutString(idBuf, name);
		m_count++;
		m_table->PutInt(kCount, m_count);
	}
	else
	{
		std::string id = m_ids[ret.first->second];
		ret.first->second = object;
		m_table->PutString(id, name);
	}
}

/**
 * Add the given object to the list of options and marks it as the default.
 * Functionally, this is very close to {@link SendableChooser#AddObject(const char *name, void *object) AddObject(...)}
 * except that it will use this as the default option if none other is explicitly selected.
 * @param name the name of the option
 * @param object the option
 */
void SendableChooser::AddDefault(const char *name, void *object)
{
	m_defaultChoice = object;
	AddObject(name, object);
	m_table->PutString(kDefault, name);
}

/**
 * Returns the selected option.  If there is none selected, it will return the default.  If there is none selected
 * and no default, then it will return {@code NULL}.
 * @return the option selected
 */
void *SendableChooser::GetSelected()
{
	return m_table->ContainsKey(kSelected) ? m_choices[m_table->GetString(kSelected)] : m_defaultChoice;
}
