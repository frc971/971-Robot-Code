/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "NetworkTables/Key.h"

#include "NetworkTables/Buffer.h"
#include "NetworkTables/Entry.h"
#include "NetworkTables/NetworkTable.h"
#include "Synchronized.h"

namespace NetworkTables
{

SEM_ID Key::_staticLock = semMCreate(SEM_Q_PRIORITY | SEM_INVERSION_SAFE | SEM_DELETE_SAFE);;
std::map<UINT32, Key *> Key::_idsMap;
UINT32 Key::_currentId = 0;

Key::Key(NetworkTable *table, const char *keyName) :
	m_table(table),
	m_name(keyName),
	m_entry(NULL),
	m_id(AllocateId())
{
	_idsMap.insert(std::pair<UINT32, Key *>(m_id, this));
}

Key::~Key()
{
	_idsMap.erase(m_id);
}

NetworkTables_Types Key::GetType()
{
	if (m_entry.get() == NULL)
		return kNetworkTables_Types_NONE;
	return m_entry->GetType();
}

void Key::Encode(Buffer *buffer)
{
	buffer->WriteByte(kNetworkTables_ASSIGNMENT);
	m_table->EncodeName(buffer);
	buffer->WriteString(m_name);
	buffer->WriteId(m_id);
}

std::auto_ptr<Entry> Key::SetEntry(std::auto_ptr<Entry> entry)
{
	Entry *old = m_entry.release();
	m_entry = entry;
	m_entry->SetKey(this);
	return std::auto_ptr<Entry>(old);
}

Key *Key::GetKey(UINT32 id)
{
	return _idsMap[id];
}

void Key::EncodeName(Buffer *buffer)
{
	buffer->WriteId(m_id);
}

UINT32 Key::AllocateId()
{
	Synchronized sync(_staticLock);
	return ++_currentId;
}

} // namespace
