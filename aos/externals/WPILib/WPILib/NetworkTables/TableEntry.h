/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __TABLE_ENTRY_H__
#define __TABLE_ENTRY_H__

#include "NetworkTables/Entry.h"
#include <vxWorks.h>

class NetworkTable;

namespace NetworkTables
{
class Buffer;

class TableEntry : public Entry
{
public:
	TableEntry(NetworkTable *value);
	virtual NetworkTables_Types GetType();
	virtual void Encode(Buffer *buffer);
	virtual NetworkTable *GetTable();

private:
	NetworkTable *m_value;
};

} // namespace

#endif
