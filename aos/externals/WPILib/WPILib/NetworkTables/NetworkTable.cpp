/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "NetworkTables/NetworkTable.h"

#include "NetworkTables/BooleanEntry.h"
#include "NetworkTables/Buffer.h"
#include "NetworkTables/Connection.h"
#include "NetworkTables/ConnectionManager.h"
#include "NetworkTables/DoubleEntry.h"
#include "NetworkTables/Entry.h"
#include "NetworkTables/IntegerEntry.h"
#include "NetworkTables/Key.h"
#include "NetworkTables/NetworkQueue.h"
#include "NetworkTables/NetworkTableChangeListener.h"
#include "NetworkTables/NetworkTableAdditionListener.h"
#include "NetworkTables/NetworkTableConnectionListener.h"
#include "NetworkTables/StringEntry.h"
#include "NetworkTables/TableEntry.h"
#include "Synchronized.h"
#include "WPIErrors.h"

NetworkTable::TableNameMap NetworkTable::_tableNameMap;
NetworkTable::TableIdMap NetworkTable::_tableIdMap;
UINT32 NetworkTable::_currentId = 1;
bool NetworkTable::_initialized = false;
SEM_ID NetworkTable::_staticMemberMutex = semMCreate(SEM_Q_PRIORITY | SEM_INVERSION_SAFE | SEM_DELETE_SAFE);

NetworkTable::NetworkTable() :
	m_dataLock(NULL),
	m_listenerLock(NULL),
	m_id(GrabId()),
	m_transaction(NULL),
	m_transactionCount(0),
	m_hasChanged(NULL),
	m_hasAdded(NULL)
{
	m_dataLock = semMCreate(SEM_Q_PRIORITY | SEM_INVERSION_SAFE | SEM_DELETE_SAFE);
	m_listenerLock = semMCreate(SEM_Q_PRIORITY | SEM_INVERSION_SAFE | SEM_DELETE_SAFE);
	m_transaction = new NetworkTables::NetworkQueue();
	m_hasChanged = new NetworkTables::NetworkQueue();
	m_hasAdded = new NetworkTables::NetworkQueue();
}

NetworkTable::~NetworkTable()
{
	delete m_hasAdded;
	delete m_hasChanged;
	delete m_transaction;
	semTake(m_listenerLock, WAIT_FOREVER);
	m_connectionListeners.clear();
	m_additionListeners.clear();
	m_listeners.clear();
	m_listenToAllListeners.clear();
	semDelete(m_listenerLock);
	semTake(m_dataLock, WAIT_FOREVER);
	m_connections.clear();
	m_data.clear();
	semDelete(m_dataLock);
}

/**
 * Opens up the connection stream.  Note that this method will be called
 * automatically when {@link NetworkTable#GetTable(const char *)} is
 * called.  This will only have an effect the first time this is called.
 */
void NetworkTable::Initialize()
{
	if (!_initialized)
	{
		_initialized = true;
		NetworkTables::ConnectionManager::GetInstance();
	}
}

/**
 * Returns the table with the given name.  The table will automatically be connected
 * by clients asking for the same table.
 * @param tableName The name of the table
 * @return The table
 */
NetworkTable *NetworkTable::GetTable(const char *tableName)
{
	Initialize();
	Synchronized sync(_staticMemberMutex);
	// Insert will add a new element if the key is not found
	//  or will return the existing key if it is found.
	std::pair<TableNameMap::iterator, bool> ret =
		_tableNameMap.insert(TableNameMap::value_type(tableName, NULL));
	if (ret.second)
		// Key not found.  Create a table.
		ret.first->second = new NetworkTable();
	return ret.first->second;
}

NetworkTable *NetworkTable::GetTable(int id)
{
	Synchronized sync(_staticMemberMutex);
	// Don't check if the id is in the map(assume it is)
	// If it's not, we will get the ID mapped to an uninitialized pointer (bad)
	// TODO: Validate success and error if not found
	return _tableIdMap[id];
}

std::vector<const char *> NetworkTable::GetKeys()
{
	Synchronized sync(m_dataLock);
	std::vector<const char *> keys;
	keys.reserve(m_data.size());
	DataMap::iterator it = m_data.begin();
	DataMap::iterator end = m_data.end();
	for (; it != end; it++)
		if (it->second->HasEntry())
			keys.push_back(it->second->GetName().c_str());
	return keys;
}

/**
 * Begins a transaction.  Note that you must call endTransaction() afterwards.
 */
void NetworkTable::BeginTransaction()
{
	Synchronized sync(m_dataLock);
	m_transactionCount++;
}

void NetworkTable::EndTransaction()
{
	Synchronized sync(m_dataLock);
	if (m_transactionCount == 0)
	{
		wpi_setWPIErrorWithContext(NetworkTablesCorrupt, "EndTransaction() called too many times");
		return;
	}
	else if (--m_transactionCount == 0)
	{
		ProcessTransaction(true, m_transaction);
	}
}

/**
 * Adds a NetworkTableChangeListener to listen to the specified element.
 * @param keyName the key to listen to
 * @param listener the listener
 * @see NetworkTableChangeListener
 */
void NetworkTable::AddChangeListener(const char *keyName, NetworkTableChangeListener *listener)
{
	Synchronized sync(m_listenerLock);
	std::set<NetworkTableChangeListener *> emptyList;
	std::pair<ListenersMap::iterator, bool> listenersForKey =
		m_listeners.insert(ListenersMap::value_type(keyName, emptyList));
	listenersForKey.first->second.insert(listener);
}

/**
 * Adds a NetworkTableChangeListener to listen to any element changed in the table
 * @param listener the listener
 * @see NetworkTableChangeListener
 */
void NetworkTable::AddChangeListenerAny(NetworkTableChangeListener *listener)
{
	Synchronized sync(m_listenerLock);
	m_listenToAllListeners.insert(listener);
}

/**
 * Removes the given NetworkTableChangeListener from the specified element.
 * @param keyName the key to no longer listen to.
 * @param listener the listener to remove
 * @see NetworkTableChangeListener
 */
void NetworkTable::RemoveChangeListener(const char *keyName,
	NetworkTableChangeListener *listener)
{
	Synchronized sync(m_listenerLock);
	ListenersMap::iterator listenersForKey = m_listeners.find(keyName);
	if (listenersForKey != m_listeners.end())
		listenersForKey->second.erase(listener);
}

/**
 * Removes the given NetworkTableChangeListener for any element in the table.
 * @param listener the listener to remove
 * @see NetworkTableChangeListener
 */
void NetworkTable::RemoveChangeListenerAny(NetworkTableChangeListener *listener)
{
	Synchronized sync(m_listenerLock);
	m_listenToAllListeners.erase(listener);
}

/**
 * Adds the NetworkTableAdditionListener to the table.
 * @param listener the listener to add
 * @see NetworkTableAdditionListener
 */
void NetworkTable::AddAdditionListener(NetworkTableAdditionListener *listener)
{
	Synchronized sync(m_listenerLock);
	m_additionListeners.insert(listener);
}

/**
 * Removes the given NetworkTableAdditionListener from the set of listeners.
 * @param listener the listener to remove
 * @see NetworkTableAdditionListener
 */
void NetworkTable::RemoveAdditionListener(
	NetworkTableAdditionListener *listener)
{
	Synchronized sync(m_listenerLock);
	m_additionListeners.erase(listener);
}

/**
 * Adds a NetworkTableConnectionListener to this table.
 * @param listener the listener to add
 * @param immediateNotify whether to tell the listener of the current connection status
 * @see NetworkTableConnectionListener
 */
void NetworkTable::AddConnectionListener(
	NetworkTableConnectionListener *listener, bool immediateNotify)
{
	Synchronized sync(m_listenerLock);
	m_connectionListeners.insert(listener);
}

/**
 * Removes the given NetworkTableConnectionListener from the table.
 * @param listener the listener to remove
 * @see NetworkTableConnectionListener
 */
void NetworkTable::RemoveConnectionListener(
	NetworkTableConnectionListener *listener)
{
	Synchronized sync(m_listenerLock);
	m_connectionListeners.erase(listener);
}

/**
 * Returns whether or not this table is connected to the robot.
 *
 * @return whether or not this table is connected to the robot
 */
bool NetworkTable::IsConnected()
{
	Synchronized sync(m_dataLock);
	return m_connections.size() > 0;
}

bool NetworkTable::ContainsKey(const char *keyName)
{
	if (keyName == NULL)
	{
		wpi_setWPIErrorWithContext(NullParameter, "keyName");
		return false;
	}
	Synchronized sync(m_dataLock);
	DataMap::iterator key = m_data.find(keyName);
	return key != m_data.end() && key->second->HasEntry();
}

/**
 * Internally used to get at the underlying Entry
 * @param keyName the name of the key
 * @return the entry at that position (or null if no entry)
 */
NetworkTables::Entry *NetworkTable::GetEntry(const char *keyName)
{
	if (keyName == NULL)
	{
		wpi_setWPIErrorWithContext(NullParameter, "keyName");
		return NULL;
	}
	Synchronized sync(m_dataLock);
	DataMap::iterator key = m_data.find(keyName);
	if (key == m_data.end())
		return NULL;
	return key->second->GetEntry();
}

/**
 * Returns the value at the specified key.
 * @param keyName the key
 * @return the value
 */
int NetworkTable::GetInt(const char *keyName)
{
	NetworkTables::Entry *entry = GetEntry(keyName);
	if (entry != NULL)
	{
		if (entry->GetType() == kNetworkTables_Types_INT)
			return entry->GetInt();
		else
			wpi_setWPIError(NetworkTablesWrongType);
	}
	return 0;
}

/**
 * Returns the value at the specified key.
 * @param keyName the key
 * @return the value
 */
bool NetworkTable::GetBoolean(const char *keyName)
{
	NetworkTables::Entry *entry = GetEntry(keyName);
	if (entry != NULL)
	{
		if (entry->GetType() == kNetworkTables_Types_BOOLEAN)
			return entry->GetBoolean();
		else
			wpi_setWPIError(NetworkTablesWrongType);
	}
	return false;
}

/**
 * Returns the value at the specified key.
 * @param keyName the key
 * @return the value
 */
double NetworkTable::GetDouble(const char *keyName)
{
	NetworkTables::Entry *entry = GetEntry(keyName);
	if (entry != NULL)
	{
		if (entry->GetType() == kNetworkTables_Types_DOUBLE)
			return entry->GetDouble();
		else
			wpi_setWPIError(NetworkTablesWrongType);
	}
	return 0.0;
}

/**
 * Returns the value at the specified key.
 * @param keyName the key
 * @return the value
 */
int NetworkTable::GetString(const char *keyName, char *value, int len)
{
	NetworkTables::Entry *entry = GetEntry(keyName);
	if (entry != NULL)
	{
		if (entry->GetType() == kNetworkTables_Types_STRING)
			return entry->GetString(value, len);
		else
			wpi_setWPIError(NetworkTablesWrongType);
	}
	return 0;
}

std::string NetworkTable::GetString(std::string keyName)
{
	NetworkTables::Entry *entry = GetEntry(keyName.c_str());
	if (entry != NULL)
	{
		if (entry->GetType() == kNetworkTables_Types_STRING)
			return entry->GetString();
		else
			wpi_setWPIError(NetworkTablesWrongType);
	}
	return "";
}


/**
 * Returns the value at the specified key.
 * @param keyName the key
 * @return the value
 */
NetworkTable *NetworkTable::GetSubTable(const char *keyName)
{
	NetworkTables::Entry *entry = GetEntry(keyName);
	if (entry != NULL)
	{
		if (entry->GetType() == kNetworkTables_Types_TABLE)
			return entry->GetTable();
		else
			wpi_setWPIError(NetworkTablesWrongType);
	}
	return NULL;
}

/**
 * Maps the specified key to the specified value in this table.
 * Neither the key nor the value can be null.
 * The value can be retrieved by calling the get method with a key that is equal to the original key.
 * @param keyName the key
 * @param value the value
 */
void NetworkTable::PutInt(const char *keyName, int value)
{
	Put(keyName, std::auto_ptr<NetworkTables::Entry>(new NetworkTables::IntegerEntry(value)));
}

/**
 * Maps the specified key to the specified value in this table.
 * Neither the key nor the value can be null.
 * The value can be retrieved by calling the get method with a key that is equal to the original key.
 * @param keyName the key
 * @param value the value
 */
void NetworkTable::PutBoolean(const char *keyName, bool value)
{
	Put(keyName, std::auto_ptr<NetworkTables::Entry>(new NetworkTables::BooleanEntry(value)));
}

/**
 * Maps the specified key to the specified value in this table.
 * Neither the key nor the value can be null.
 * The value can be retrieved by calling the get method with a key that is equal to the original key.
 * @param keyName the key
 * @param value the value
 */
void NetworkTable::PutDouble(const char *keyName, double value)
{
	Put(keyName, std::auto_ptr<NetworkTables::Entry>(new NetworkTables::DoubleEntry(value)));
}

/**
 * Maps the specified key to the specified value in this table.
 * Neither the key nor the value can be null.
 * The value can be retrieved by calling the get method with a key that is equal to the original key.
 * @param keyName the key
 * @param value the value
 */
void NetworkTable::PutString(const char *keyName, const char *value)
{
	if (value == NULL)
	{
		wpi_setWPIErrorWithContext(NullParameter, "value");
		return;
	}
	Put(keyName, std::auto_ptr<NetworkTables::Entry>(new NetworkTables::StringEntry(value)));
}

void NetworkTable::PutString(std::string keyName, std::string value)
{
	PutString(keyName.c_str(), value.c_str());
}

/**
 * Maps the specified key to the specified value in this table.
 * Neither the key nor the value can be null.
 * The value can be retrieved by calling the get method with a key that is equal to the original key.
 * @param keyName the key
 * @param value the value
 */
void NetworkTable::PutSubTable(const char *keyName, NetworkTable *value)
{
	if (value == NULL)
	{
		wpi_setWPIErrorWithContext(NullParameter, "value");
		return;
	}
	Put(keyName, std::auto_ptr<NetworkTables::Entry>(new NetworkTables::TableEntry(value)));
}

UINT32 NetworkTable::GrabId()
{
	Synchronized sync(_staticMemberMutex);
	return _currentId++;
}

void NetworkTable::ProcessTransaction(bool confirmed, NetworkTables::NetworkQueue *transaction)
{
	if (transaction->IsEmpty())
		return;

	NetworkTables::Connection *source = ((NetworkTables::Entry *)transaction->Peek())->GetSource();

	Synchronized sync(m_dataLock);

	std::set<NetworkTables::Connection *>::iterator it, end;
	it = m_connections.begin();
	end = m_connections.end();
	for (; it != end; it++)
		if (*it != source)
			(*it)->OfferTransaction(transaction);

	while (!transaction->IsEmpty())
	{
		std::pair<NetworkTables::Data *, bool> data = transaction->Poll();
		// TODO: Remove this
		if (!data.second)
			printf("Internal error!");
		std::auto_ptr<NetworkTables::Entry> entry =
			std::auto_ptr<NetworkTables::Entry>((NetworkTables::Entry *)data.first);
		std::auto_ptr<NetworkTables::Entry> oldEntry = entry->GetKey()->SetEntry(entry);
		if (oldEntry.get() == NULL)
			m_hasAdded->Offer(data.first);
		else	// TODO: Filter unchanged values
			m_hasChanged->Offer(data.first);
	}
	while (!m_hasAdded->IsEmpty())
	{
		NetworkTables::Entry *entry = (NetworkTables::Entry *)m_hasAdded->Poll().first;
		AlertListeners(true, confirmed, entry->GetKey()->GetName().c_str(), entry);
	}
	while (!m_hasChanged->IsEmpty())
	{
		NetworkTables::Entry *entry = (NetworkTables::Entry *)m_hasChanged->Poll().first;
		AlertListeners(true, confirmed, entry->GetKey()->GetName().c_str(), entry);
	}
}

void NetworkTable::AddConnection(NetworkTables::Connection *connection)
{
	Synchronized sync(m_dataLock);

	if (m_connections.insert(connection).second)
	{
		DataMap::iterator it = m_data.begin();
		DataMap::iterator end = m_data.end();
		for (; it != end; it++)
		{
			connection->Offer(it->second);
			if(it->second->HasEntry())
				connection->Offer(it->second->GetEntry());
		}
		if (m_connections.size() == 1)
		{
			Synchronized syncStatic(_staticMemberMutex);
			_tableIdMap.insert(TableIdMap::value_type(m_id, this));

			std::set<NetworkTableConnectionListener *>::iterator lit, lend;
			lit = m_connectionListeners.begin();
			lend = m_connectionListeners.end();
			for (; lit != lend; lit++)
				(*lit)->Connected(this);
		}
	}
}

void NetworkTable::RemoveConnection(NetworkTables::Connection *connection)
{
	Synchronized sync(m_dataLock);

	m_connections.erase(connection);

	if (m_connections.size() == 0)
	{
		Synchronized syncStatic(_staticMemberMutex);
		_tableIdMap.erase(m_id);

		std::set<NetworkTableConnectionListener *>::iterator lit, lend;
		lit = m_connectionListeners.begin();
		lend = m_connectionListeners.end();
		for (; lit != lend; lit++)
			(*lit)->Disconnected(this);
	}
}

/**
 * Returns the key that the name maps to.  This should
 * never fail, if their is no key for that name, then one should be made.
 * @param keyName the name
 * @return the key
 */
NetworkTables::Key *NetworkTable::GetKey(const char *keyName)
{
	Synchronized sync(m_dataLock);
	// Insert will add a new element if the key is not found
	//  or will return the existing key if it is found.
	std::pair<DataMap::iterator, bool> ret =
		m_data.insert(DataMap::value_type(keyName, NULL));
	if (ret.second)
	{
		// Key not found.  Create a new one.
		ret.first->second = new NetworkTables::Key(this, keyName);
		if (m_connections.size() != 0)
		{
			std::set<NetworkTables::Connection *>::iterator it, end;
			it = m_connections.begin();
			end = m_connections.end();
			for (; it != end; it++)
				(*it)->Offer(ret.first->second);
		}
	}
	return ret.first->second;
}

void NetworkTable::Put(const char *keyName, std::auto_ptr<NetworkTables::Entry> value)
{
	if (keyName == NULL)
	{
		wpi_setWPIErrorWithContext(NullParameter, "keyName");
		return;
	}
	Synchronized sync(m_dataLock);
	NetworkTables::Key *key = GetKey(keyName);
	value->SetKey(key);

	if (m_transactionCount == 0)
		Got(true, key, value);
	else
		m_transaction->Offer(std::auto_ptr<NetworkTables::Data>(value.release()));
}

void NetworkTable::Send(NetworkTables::Entry *entry)
{
	Synchronized sync(m_dataLock);
	std::set<NetworkTables::Connection *>::iterator it, end;
	it = m_connections.begin();
	end = m_connections.end();
	for (; it != end; it++)
		if (*it != entry->GetSource())
			(*it)->Offer(entry);
}

/**
 * This method should be called by children when they want to add a new value.
 * It will notify listeners of the value
 * @param confirmed whether or not this value was confirmed or received
 * @param key the key
 * @param value the value
 */
void NetworkTable::Got(bool confirmed, NetworkTables::Key *key, std::auto_ptr<NetworkTables::Entry> value)
{
	std::auto_ptr<NetworkTables::Entry> old;
	{
		Synchronized sync(m_dataLock);
		old = key->SetEntry(value);
	}
	// TODO: return if value didn't change

	Send(key->GetEntry());
	AlertListeners(old.get() == NULL, confirmed, key->GetName().c_str(), key->GetEntry());
}

void NetworkTable::AlertListeners(bool isNew, bool confirmed, const char *keyName,
	NetworkTables::Entry *value)
{
	Synchronized sync(m_listenerLock);

	if (isNew && m_additionListeners.size() != 0)
	{
		std::set<NetworkTableAdditionListener *>::iterator lit, lend;
		lit = m_additionListeners.begin();
		lend = m_additionListeners.end();
		for (; lit != lend; lit++)
			(*lit)->FieldAdded(this, keyName, value->GetType());
	}

	ListenersMap::iterator listeners = m_listeners.find(keyName);
	if (listeners != m_listeners.end())
	{
		std::set<NetworkTableChangeListener *>::iterator lit, lend;
		lit = listeners->second.begin();
		lend = listeners->second.end();
		for (; lit != lend; lit++)
		{
			if (confirmed)
				(*lit)->ValueConfirmed(this, keyName, value->GetType());
			else
				(*lit)->ValueChanged(this, keyName, value->GetType());
		}
	}

	{
		std::set<NetworkTableChangeListener *>::iterator lit, lend;
		lit = m_listenToAllListeners.begin();
		lend = m_listenToAllListeners.end();
		for (; lit != lend; lit++)
		{
			if (confirmed)
				(*lit)->ValueConfirmed(this, keyName, value->GetType());
			else
				(*lit)->ValueChanged(this, keyName, value->GetType());
		}
	}
}

void NetworkTable::EncodeName(NetworkTables::Buffer *buffer)
{
	buffer->WriteTableId(m_id);
}
