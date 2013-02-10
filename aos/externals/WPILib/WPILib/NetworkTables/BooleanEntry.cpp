/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "NetworkTables/BooleanEntry.h"
#include "NetworkTables/Buffer.h"

namespace NetworkTables
{

BooleanEntry::BooleanEntry(bool value) :
	m_value(value)
{
}

NetworkTables_Types BooleanEntry::GetType()
{
	return kNetworkTables_Types_BOOLEAN;
}

void BooleanEntry::Encode(Buffer *buffer)
{
	Entry::Encode(buffer);
	buffer->WriteByte(m_value ? kNetworkTables_BOOLEAN_TRUE
		: kNetworkTables_BOOLEAN_FALSE);
}

bool BooleanEntry::GetBoolean()
{
	return m_value;
}

} // namespace
