/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __BOOLEAN_ENTRY_H__
#define __BOOLEAN_ENTRY_H__

#include "NetworkTables/Entry.h"
#include "NetworkTables/InterfaceConstants.h"
#include <vxWorks.h>

class NetworkTable;

namespace NetworkTables
{

class Buffer;

class BooleanEntry : public Entry {
public:
	BooleanEntry(bool value);
	virtual NetworkTables_Types GetType();
	virtual void Encode(Buffer *buffer);
	virtual bool GetBoolean();

private:
	bool m_value;
};

} // namespace

#endif
