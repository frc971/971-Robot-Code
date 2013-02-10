/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __KEY_H__
#define __KEY_H__

#include "NetworkTables/Data.h"
#include "NetworkTables/InterfaceConstants.h"

#include <map>
#include <string>

class NetworkTable;

namespace NetworkTables
{

class Entry;

class Key : public Data
{
	friend class Connection;
	friend class Entry;
	friend class KeyConnectionListener;
	friend class NetworkTable;
public:
	Key(NetworkTable *table, const char *keyName);
	virtual ~Key();
	NetworkTable *GetTable() {return m_table;}
	NetworkTables_Types GetType();
	Entry *GetEntry() {return m_entry.get();}
	std::string GetName() {return m_name;}
	UINT32 GetId() {return m_id;}
	void Encode(Buffer *buffer);

	static Key *GetKey(UINT32 id);

private:
	std::auto_ptr<Entry> SetEntry(std::auto_ptr<Entry> entry);
	bool HasEntry() {return m_entry.get() != NULL;}
	void EncodeName(Buffer *buffer);

	static UINT32 AllocateId();

	NetworkTable *m_table;
	std::string m_name;
	// Keys are responsible for entrys' memory
	std::auto_ptr<Entry> m_entry;
	UINT32 m_id;

	static SEM_ID _staticLock;
	static std::map<UINT32, Key *> _idsMap;
	static UINT32 _currentId;
};

} // namespace

#endif

