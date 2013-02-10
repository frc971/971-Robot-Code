/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "NetworkTables/OldData.h"
#include "NetworkTables/Buffer.h"
#include "NetworkTables/Entry.h"

namespace NetworkTables
{

OldData::OldData(Entry *entry) :
	m_entry(entry)
{
}

void OldData::Encode(Buffer *buffer)
{
	buffer->WriteByte(kNetworkTables_OLD_DATA);
	m_entry->Encode(buffer);
}

} // namespace
