/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "NetworkTables/Entry.h"
#include "NetworkTables/Key.h"
#include "WPIErrors.h"

namespace NetworkTables
{

UINT32 Entry::GetId()
{
	return m_key->GetId();
}

void Entry::Encode(Buffer *buffer)
{
	GetKey()->EncodeName(buffer);
}

int Entry::GetInt()
{
	wpi_setWPIError(NetworkTablesWrongType);
	return 0;
}

double Entry::GetDouble()
{
	wpi_setWPIError(NetworkTablesWrongType);
	return 0.0;
}

bool Entry::GetBoolean()
{
	wpi_setWPIError(NetworkTablesWrongType);
	return false;
}

int Entry::GetString(char *str, int len)
{
	wpi_setWPIError(NetworkTablesWrongType);
	return 0;
}

std::string Entry::GetString()
{
	wpi_setWPIError(NetworkTablesWrongType);
	return "";
}

NetworkTable *Entry::GetTable()
{
	wpi_setWPIError(NetworkTablesWrongType);
	return NULL;
}

} // namespace
