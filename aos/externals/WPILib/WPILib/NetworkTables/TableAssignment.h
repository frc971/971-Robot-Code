/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __TABLE_ASSIGNMENT_H__
#define __TABLE_ASSIGNMENT_H__

#include "NetworkTables/Data.h"

class NetworkTable;

namespace NetworkTables
{

class Buffer;

class TableAssignment : public Data
{
public:
	TableAssignment(NetworkTable *table, int alteriorId);
	virtual void Encode(Buffer *buffer);

private:
	NetworkTable *m_table;
	int m_alteriorId;
};

} // namespace

#endif
