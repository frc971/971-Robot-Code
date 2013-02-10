/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "NetworkTables/IntegerEntry.h"
#include "NetworkTables/Buffer.h"

namespace NetworkTables
{

IntegerEntry::IntegerEntry(int value) :
	m_value(value)
{
}

NetworkTables_Types IntegerEntry::GetType()
{
	return kNetworkTables_Types_INT;
}

void IntegerEntry::Encode(Buffer *buffer)
{
	Entry::Encode(buffer);
	buffer->WriteByte(kNetworkTables_INT);
	buffer->WriteInt(m_value);
}

int IntegerEntry::GetInt()
{
	return m_value;
}

} // namespace
