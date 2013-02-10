/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __OLD_DATA_H__
#define __OLD_DATA_H__

#include "NetworkTables/Data.h"
#include <vxWorks.h>

namespace NetworkTables
{
class Buffer;
class Entry;

class OldData : public Data
{
public:
	OldData(Entry *entry);
	virtual void Encode(Buffer *buffer);
	virtual bool IsOldData() {return true;}
	Entry *GetEntry() {return m_entry;}

private:
	Entry *m_entry;
};

} // namespace

#endif
