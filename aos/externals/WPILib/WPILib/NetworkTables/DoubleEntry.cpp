/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "NetworkTables/DoubleEntry.h"
#include "NetworkTables/Buffer.h"

namespace NetworkTables
{

DoubleEntry::DoubleEntry(double value) :
	m_value(value)
{
}

NetworkTables_Types DoubleEntry::GetType()
{
	return kNetworkTables_Types_DOUBLE;
}

void DoubleEntry::Encode(Buffer *buffer)
{
	Entry::Encode(buffer);
	buffer->WriteByte(kNetworkTables_DOUBLE);
	buffer->WriteDouble(m_value);
}

double DoubleEntry::GetDouble()
{
	return m_value;
}

} // namespace
