/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "NetworkTables/StringEntry.h"
#include "NetworkTables/Buffer.h"

namespace NetworkTables
{

StringEntry::StringEntry(const char *str)
{
	m_value = str;
}

NetworkTables_Types StringEntry::GetType()
{
	return kNetworkTables_Types_STRING;
}

void StringEntry::Encode(Buffer *buffer)
{
	Entry::Encode(buffer);
	buffer->WriteByte(kNetworkTables_STRING);
	buffer->WriteString(m_value.c_str());
}

int StringEntry::GetString(char *str, int len)
{
	return (int)m_value.copy(str, len);
}

std::string StringEntry::GetString()
{
	return m_value;
}

} // namespace
