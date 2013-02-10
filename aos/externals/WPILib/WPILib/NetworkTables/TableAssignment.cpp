/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "NetworkTables/TableAssignment.h"
#include "NetworkTables/Buffer.h"
#include "NetworkTables/InterfaceConstants.h"
#include "NetworkTables/NetworkTable.h"

namespace NetworkTables
{

TableAssignment::TableAssignment(NetworkTable *table, int alteriorId) :
	m_table(table),
	m_alteriorId(alteriorId)
{
}

void TableAssignment::Encode(Buffer *buffer)
{
	buffer->WriteByte(kNetworkTables_TABLE_ASSIGNMENT);
	buffer->WriteTableId(m_alteriorId);
	m_table->EncodeName(buffer);
}

} // namespace
