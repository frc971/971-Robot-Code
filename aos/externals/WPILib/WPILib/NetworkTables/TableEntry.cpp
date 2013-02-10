/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "NetworkTables/TableEntry.h"
#include "NetworkTables/InterfaceConstants.h"
#include "NetworkTables/NetworkTable.h"

namespace NetworkTables
{

TableEntry::TableEntry(NetworkTable *value) :
	m_value(value)
{
}

NetworkTables_Types TableEntry::GetType()
{
	return kNetworkTables_Types_TABLE;
}

void TableEntry::Encode(Buffer *buffer)
{
	Entry::Encode(buffer);
	m_value->EncodeName(buffer);
}

NetworkTable *TableEntry::GetTable()
{
	return m_value;
}

} // namespace
