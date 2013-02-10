/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __NETWORK_TABLE_H__
#define __NETWORK_TABLE_H__

#include "ErrorBase.h"
#include <map>
#include <set>
#include <vector>

namespace NetworkTables
{
    class Buffer;
    class Connection;
    class Entry;
    class Key;
    class NetworkQueue;
    class TableAssignment;
    class TableEntry;
}

class NetworkTableChangeListener;
class NetworkTableAdditionListener;
class NetworkTableConnectionListener;

class NetworkTable : public ErrorBase {
	friend class NetworkTables::Connection;
	friend class NetworkTables::Key;
	friend class NetworkTables::TableAssignment;
	friend class NetworkTables::TableEntry;
public:
	NetworkTable();
	~NetworkTable();
	static void Initialize();
	static NetworkTable *GetTable(const char *tableName);
	static NetworkTable *GetTable(int id);
	std::vector<const char *> GetKeys();
	void BeginTransaction();
	void EndTransaction();
	void AddChangeListener(const char *keyName, NetworkTableChangeListener *listener);
	void AddChangeListenerAny(NetworkTableChangeListener *listener);
	void RemoveChangeListener(const char *keyName, NetworkTableChangeListener *listener);
	void RemoveChangeListenerAny(NetworkTableChangeListener *listener);
	void AddAdditionListener(NetworkTableAdditionListener *listener);
	void RemoveAdditionListener(NetworkTableAdditionListener *listener);
	void AddConnectionListener(NetworkTableConnectionListener *listener, bool immediateNotify);
	void RemoveConnectionListener(NetworkTableConnectionListener *listener);
	bool IsConnected();
	bool ContainsKey(const char *keyName);
	NetworkTables::Entry *GetEntry(const char *keyName);
	
	int GetInt(const char *keyName);
	bool GetBoolean(const char *keyName);
	double GetDouble(const char *keyName);
	int GetString(const char *keyName, char *value, int len);
	std::string GetString(std::string keyName);
	NetworkTable *GetSubTable(const char *keyName);
	void PutInt(const char *keyName, int value);
	void PutBoolean(const char *keyName, bool value);
	void PutDouble(const char *keyName, double value);
	void PutString(const char *keyName, const char *value);
	void PutString(std::string keyName, std::string value);
	void PutSubTable(const char *keyName, NetworkTable *value);
	
private:
	static UINT32 GrabId();
	void ProcessTransaction(bool confirmed, NetworkTables::NetworkQueue *transaction);
	UINT32 GetId() {return m_id;}
	void AddConnection(NetworkTables::Connection *connection);
	void RemoveConnection(NetworkTables::Connection *connection);
	NetworkTables::Key *GetKey(const char *keyName);
	void Put(const char *keyName, std::auto_ptr<NetworkTables::Entry> value);
	void Send(NetworkTables::Entry *entry);
	void Got(bool confirmed, NetworkTables::Key *key, std::auto_ptr<NetworkTables::Entry> value);
	void AlertListeners(bool isNew, bool confirmed, const char *keyName, NetworkTables::Entry *value);
	void EncodeName(NetworkTables::Buffer *buffer);

	/** The lock for data */
	SEM_ID m_dataLock;
	/** The actual data */
	typedef std::map<std::string, NetworkTables::Key *> DataMap;
	DataMap m_data;
	/** The connections this table has */
	std::set<NetworkTables::Connection *> m_connections;
	/** The lock for listener modification */
	SEM_ID m_listenerLock;
	/** Set of NetworkingListeners who register for everything */
	std::set<NetworkTableChangeListener *> m_listenToAllListeners;
	/** Links names to NetworkingListeners */
	typedef std::map<std::string, std::set<NetworkTableChangeListener *> > ListenersMap;
	ListenersMap m_listeners;
	/** Set of addition listeners */
	std::set<NetworkTableAdditionListener *> m_additionListeners;
	/** Set of connection listeners */
	std::set<NetworkTableConnectionListener *> m_connectionListeners;
	/** The id of this table */
	UINT32 m_id;
	/** The queue of the current transaction */
	NetworkTables::NetworkQueue *m_transaction;
	/** The number of times begin transaction has been called without a matching end transaction */
	int m_transactionCount;
	/** A list of values which need to be signaled */
	NetworkTables::NetworkQueue *m_hasChanged;
	/** A list of values which has been added */
	NetworkTables::NetworkQueue *m_hasAdded;

	/** Links names to tables */
	typedef std::map<std::string, NetworkTable *> TableNameMap;
	static TableNameMap _tableNameMap;
	/** Links ids to currently active NetworkingTables */
	typedef std::map<UINT32, NetworkTable *> TableIdMap;
	static TableIdMap _tableIdMap;
	/** The currently available id */
	static UINT32 _currentId;
	/** Indicates that static variables are initialized */
	static bool _initialized;
	/** Protects access to static members */
	static SEM_ID _staticMemberMutex;
	/** Usage Guidelines... */
	DISALLOW_COPY_AND_ASSIGN(NetworkTable);
};

#endif // __NETWORK_TABLE_H__
