/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __STRING_ENTRY_H__
#define __STRING_ENTRY_H__

#include "NetworkTables/Entry.h"
#include "NetworkTables/InterfaceConstants.h"
#include <vxWorks.h>
#include <string>

class NetworkTable;

namespace NetworkTables
{

class Buffer;

class StringEntry : public Entry {
public:
	StringEntry(const char *str);
	virtual ~StringEntry() {}
	virtual NetworkTables_Types GetType();
	virtual void Encode(Buffer *buffer);
	virtual int GetString(char *str, int len);
	virtual std::string GetString();

private:
	std::string m_value;
};

} // namespace

#endif
