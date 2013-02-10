/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "Buttons/NetworkButton.h"

#include "NetworkTables/NetworkTable.h"

NetworkButton::NetworkButton(const char *tableName, const char *field) :
	m_netTable(NetworkTable::GetTable(tableName)),
	m_field(field)
{
}

NetworkButton::NetworkButton(NetworkTable *table, const char *field) :
	m_netTable(table),
	m_field(field)
{
}

bool NetworkButton::Get()
{
	if (m_netTable->IsConnected())
		return m_netTable->GetBoolean(m_field.c_str());
	else
		return false;
}
