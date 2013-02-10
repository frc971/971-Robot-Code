/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __ENTRY_H__
#define __ENTRY_H__

#include "ErrorBase.h"
#include "NetworkTables/Data.h"
#include "NetworkTables/InterfaceConstants.h"
#include <vxWorks.h>

class NetworkTable;

namespace NetworkTables
{

class Buffer;
class Connection;
class Key;

class Entry : public Data, public ErrorBase
{
public:
	void SetKey(Key *key) {m_key = key;}
	Key *GetKey() {return m_key;}
	UINT32 GetId();
    void SetSource(Connection *source) {m_source = source;}
	Connection *GetSource() {return m_source;}

	virtual void Encode(Buffer *buffer);
	virtual bool IsEntry() {return true;}

	virtual NetworkTables_Types GetType() = 0;
	virtual int GetInt();
	virtual double GetDouble();
	virtual bool GetBoolean();
	virtual int GetString(char *str, int len);
	virtual std::string GetString();
	virtual NetworkTable *GetTable();

private:

	Key *m_key;
	Connection *m_source;
};

} // namespace

#endif
