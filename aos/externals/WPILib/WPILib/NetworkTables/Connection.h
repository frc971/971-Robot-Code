/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __CONNECTION_H__
#define __CONNECTION_H__

#include "ErrorBase.h"
#include "Task.h"
#include <map>
#include <deque>

class NetworkTable;

namespace NetworkTables
{
class Data;
class Entry;
class Key;
class NetworkQueue;
class TransactionEnd;
class TransactionStart;

class Connection : public ErrorBase
{
	friend class ConnectionManager;
	friend class NetworkTable;
	friend class Reader;
public:
	static const UINT32 kWriteDelay = 250;
	static const UINT32 kTimeout = 1000;

private:
	Connection(int socket);
	~Connection();
	void OfferTransaction(NetworkQueue *transaction);
	void Offer(Data *data);
	void Offer(std::auto_ptr<Data> autoData);
	void Start();
	void ReadTaskRun();
	void WriteTaskRun();
	void Close();
	bool IsConnected() {return m_connected;}
	NetworkTable *GetTable(bool local, UINT32 id);
	bool ConfirmationsContainsKey(Key *key);
	void WatchdogActivate();
	void WatchdogFeed();
	void WatchdogTaskRun();

	static int InitReadTask(Connection *obj) {obj->ReadTaskRun();return 0;}
	static int InitWriteTask(Connection *obj) {obj->WriteTaskRun();return 0;}
	static int InitWatchdogTask(Connection *obj) {obj->WatchdogTaskRun();return 0;}

	int m_socket;
	SEM_ID m_dataLock;
	SEM_ID m_dataAvailable;
	SEM_ID m_watchdogLock;
	SEM_ID m_watchdogFood;
	typedef std::map<UINT32, UINT32> IDMap_t;
	IDMap_t m_tableMap;
	IDMap_t m_fieldMap;
	NetworkQueue *m_queue;
	std::deque<Entry *> m_confirmations;
	NetworkQueue *m_transaction;
	bool m_connected;
	bool m_inTransaction;
	bool m_denyTransaction;
	bool m_watchdogActive;
	bool m_watchdogFed;
	Task m_readTask;
	Task m_writeTask;
	Task m_watchdogTask;
	TransactionStart *m_transactionStart;
	TransactionEnd *m_transactionEnd;
};

} // namespace

#endif

