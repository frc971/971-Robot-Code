/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "NetworkTables/Connection.h"

#include "NetworkTables/Buffer.h"
#include "NetworkTables/Confirmation.h"
#include "NetworkTables/ConnectionManager.h"
#include "NetworkTables/Data.h"
#include "NetworkTables/Denial.h"
#include "NetworkTables/Entry.h"
#include "NetworkTables/InterfaceConstants.h"
#include "NetworkTables/Key.h"
#include "NetworkTables/NetworkQueue.h"
#include "NetworkTables/NetworkTable.h"
#include "NetworkTables/OldData.h"
#include "NetworkTables/Reader.h"
#include "NetworkTables/TableAssignment.h"
#include "NetworkTables/TableEntry.h"
#include "NetworkTables/TransactionEnd.h"
#include "NetworkTables/TransactionStart.h"
#include "Synchronized.h"
#include "Timer.h"
#include "WPIErrors.h"
#include <inetLib.h>
#include <semLib.h>
#include <sockLib.h>
#include <string>
#include <usrLib.h>

namespace NetworkTables
{

const UINT32 Connection::kWriteDelay;
const UINT32 Connection::kTimeout;

Connection::Connection(int socket) :
	m_socket(socket),
	m_dataLock(NULL),
	m_dataAvailable(NULL),
	m_watchdogLock(NULL),
	m_watchdogFood(NULL),
	m_queue(NULL),
	m_transaction(NULL),
	m_connected(true),
	m_inTransaction(false),
	m_denyTransaction(false),
	m_watchdogActive(false),
	m_watchdogFed(false),
	m_readTask("NetworkTablesReadTask", (FUNCPTR)Connection::InitReadTask),
	m_writeTask("NetworkTablesWriteTask", (FUNCPTR)Connection::InitWriteTask),
	m_watchdogTask("NetworkTablesWatchdogTask", (FUNCPTR)Connection::InitWatchdogTask),
	m_transactionStart(NULL),
	m_transactionEnd(NULL)
{
	m_dataLock = semMCreate(SEM_Q_PRIORITY | SEM_INVERSION_SAFE | SEM_DELETE_SAFE);
	m_dataAvailable = semBCreate (SEM_Q_PRIORITY, SEM_EMPTY);
	m_watchdogLock = semMCreate(SEM_Q_PRIORITY | SEM_INVERSION_SAFE | SEM_DELETE_SAFE);
	m_watchdogFood = semBCreate (SEM_Q_PRIORITY, SEM_EMPTY);
	m_queue = new NetworkQueue();
	m_transaction = new NetworkQueue();
	m_transactionStart = new TransactionStart();
	m_transactionEnd = new TransactionEnd();
}

Connection::~Connection()
{
	delete m_transactionEnd;
	delete m_transactionStart;
	delete m_transaction;
	delete m_queue;
	semDelete(m_watchdogFood);
	semTake(m_watchdogLock, WAIT_FOREVER);
	semDelete(m_watchdogLock);
	semDelete(m_dataAvailable);
	semTake(m_dataLock, WAIT_FOREVER);
	semDelete(m_dataLock);
}

void Connection::OfferTransaction(NetworkQueue *transaction)
{
	Synchronized sync(m_dataLock);
	NetworkQueue::DataQueue_t::const_iterator it = transaction->GetQueueHead();
	for (; !transaction->IsQueueEnd(it); it++)
	{
		Data *data = it->first;
		if (data->IsEntry() && ((Entry *)data)->GetType() == kNetworkTables_TABLE)
		{
			NetworkTable *table = ((TableEntry *)data)->GetTable();
			table->AddConnection(this);
		}
	}
	m_queue->Offer(m_transactionStart);
	it = transaction->GetQueueHead();
	for (; !transaction->IsQueueEnd(it); it++)
	{
		// We are always offering something from a transaction that has yet to be used locally...
		// They will be cleaned up if necessary in the local use.
		m_queue->Offer(it->first);
	}
	m_queue->Offer(m_transactionEnd);
	semGive(m_dataAvailable);
}

void Connection::Offer(Data *data)
{
	if (data != NULL)
	{
		Synchronized sync(m_dataLock);
		if (data->IsEntry() && ((Entry *)data)->GetType() == kNetworkTables_TABLE)
		{
			NetworkTable *table = ((TableEntry *)data)->GetTable();
			table->AddConnection(this);
		}
		m_queue->Offer(data);
		semGive(m_dataAvailable);
	}
}

void Connection::Offer(std::auto_ptr<Data> autoData)
{
	Synchronized sync(m_dataLock);
	if (autoData->IsEntry() && ((Entry *)autoData.get())->GetType() == kNetworkTables_TABLE)
	{
		NetworkTable *table = ((TableEntry *)autoData.get())->GetTable();
		table->AddConnection(this);
	}
	m_queue->Offer(autoData);
	semGive(m_dataAvailable);
}

void Connection::Start()
{
	m_watchdogTask.Start((UINT32)this);
	m_readTask.Start((UINT32)this);
	m_writeTask.Start((UINT32)this);
}

void Connection::ReadTaskRun()
{
	Reader input(this, m_socket);
	int value;
	
	value = input.Read();
	while (m_connected)
	{
		WatchdogFeed();
		WatchdogActivate();

		if (value >= kNetworkTables_ID || value == kNetworkTables_OLD_DATA)
		{
			bool oldData = value == kNetworkTables_OLD_DATA;
			UINT32 id = input.ReadId(!oldData);
			if (!m_connected)
				break;
			Key *key = Key::GetKey(m_fieldMap[id]);
#ifdef DEBUG
			char pbuf[64];
			snprintf(pbuf, 64, "Update field \"%s\" value remote=%d local=%d\n", key->GetName().c_str(), id, m_fieldMap[id]);
			printf(pbuf);
#endif

			if (key == NULL)
			{
				wpi_setWPIErrorWithContext(NetworkTablesCorrupt, "Unexpected ID");
				Close();
				return;
			}

			value = input.Read();
			if (!m_connected)
				break;

			if (ConnectionManager::GetInstance()->IsServer() && ConfirmationsContainsKey(key))
			{
				if (m_inTransaction)
					m_denyTransaction = true;
				else
					Offer(std::auto_ptr<Data>(new Denial(1)));
				if (value >= kNetworkTables_TABLE_ID)
					input.ReadTableId(true);
				else
					input.ReadEntry(true);
			}
			else if (value >= kNetworkTables_TABLE_ID)
			{
				UINT32 tableId = input.ReadTableId(true);
				if (!m_connected)
					break;
				if (oldData && key->HasEntry())
				{
					Offer(std::auto_ptr<Data>(new Denial(1)));
				}
				else
				{
					NetworkTable *table = GetTable(false, tableId);
					Entry *tableEntry = new TableEntry(table);
					tableEntry->SetSource(this);
					tableEntry->SetKey(key);
					if (m_inTransaction)
					{
						m_transaction->Offer(std::auto_ptr<Data>(tableEntry));
					}
					else
					{
						key->GetTable()->Got(false, key, std::auto_ptr<Entry>(tableEntry));
						Offer(std::auto_ptr<Data>(new Confirmation(1)));
					}
				}
			}
			else
			{
				std::auto_ptr<Entry> entry = input.ReadEntry(true);
				if (!m_connected)
					break;

				if (entry.get() == NULL)
				{
					wpi_setWPIErrorWithContext(NetworkTablesCorrupt, "Unable to parse entry");
					Close();
					return;
				}
				else if (oldData && key->HasEntry())
				{
					Offer(std::auto_ptr<Data>(new Denial(1)));
				}
				else
				{
					entry->SetSource(this);
					entry->SetKey(key);
					if (m_inTransaction)
					{
						m_transaction->Offer(std::auto_ptr<Data>(entry.release()));
					}
					else
					{
						key->GetTable()->Got(false, key, entry);
						Offer(std::auto_ptr<Data>(new Confirmation(1)));
					}
				}
			}
		}
		else if (value >= kNetworkTables_CONFIRMATION)
		{
			int count = input.ReadConfirmations(true);
			if (!m_connected)
				break;
			while (count-- > 0)
			{
				if (m_confirmations.empty())
				{
					wpi_setWPIErrorWithContext(NetworkTablesCorrupt, "Too many confirmations");
					Close();
					return;
				}
				Entry *entry = m_confirmations.front();
				m_confirmations.pop_front();
				// TransactionStart
				if (entry == NULL)
				{
					if (ConnectionManager::GetInstance()->IsServer())
					{
						while (!m_confirmations.empty() && m_confirmations.front() != NULL)
							m_confirmations.pop_front();
					}
					else
					{
						while (!m_confirmations.empty() && m_confirmations.front() != NULL)
						{
							m_transaction->Offer(m_confirmations.front());
							m_confirmations.pop_front();
						}

						if (!m_transaction->IsEmpty())
							((Entry *)m_transaction->Peek())->GetKey()->GetTable()->ProcessTransaction(true, m_transaction);
					}
				}
				else if (!ConnectionManager::GetInstance()->IsServer())
				{
					entry->GetKey()->GetTable()->Got(true, entry->GetKey(), std::auto_ptr<Entry>(entry));
				}
			}
		}
		else if (value >= kNetworkTables_DENIAL)
		{
			if (ConnectionManager::GetInstance()->IsServer())
			{
				wpi_setWPIErrorWithContext(NetworkTablesCorrupt, "Server can not be denied");
				Close();
				return;
			}
			int count = input.ReadDenials(m_connected);
			if (!m_connected)
				break;
			while (count-- > 0)
			{
				if (m_confirmations.empty())
				{
					wpi_setWPIErrorWithContext(NetworkTablesCorrupt, "Excess denial");
					Close();
					return;
				}
				else if (m_confirmations.front() == NULL)
				{
					m_confirmations.pop_front();
					// Skip the transaction
					while (!m_confirmations.empty() && m_confirmations.front() != NULL)
					{
						delete m_confirmations.front();
						m_confirmations.pop_front();
					}
				}
				else
				{
					delete m_confirmations.front();
					m_confirmations.pop_front();
				}
			}
		}
		else if (value == kNetworkTables_TABLE_REQUEST)
		{
			if (!ConnectionManager::GetInstance()->IsServer())
			{
				wpi_setWPIErrorWithContext(NetworkTablesCorrupt, "Server requesting table");
				Close();
				return;
			}
			std::string name = input.ReadString();
			if (!m_connected)
				break;
			UINT32 id = input.ReadTableId(false);
			if (!m_connected)
				break;
#ifdef DEBUG
			char pbuf[128];
			snprintf(pbuf, 128, "Request table: %s (%d)\n", name.c_str(), id);
			printf(pbuf);
#endif

			NetworkTable *table = NetworkTable::GetTable(name.c_str());

			{
				Synchronized sync(m_dataLock);
				Offer(std::auto_ptr<Data>(new TableAssignment(table, id)));
				table->AddConnection(this);
			}

			m_tableMap.insert(IDMap_t::value_type(id, table->GetId()));
		}
		else if (value == kNetworkTables_TABLE_ASSIGNMENT)
		{
			UINT32 localTableId = input.ReadTableId(false);
			if (!m_connected)
				break;
			UINT32 remoteTableId = input.ReadTableId(false);
			if (!m_connected)
				break;
#ifdef DEBUG
			char pbuf[64];
			snprintf(pbuf, 64, "Table Assignment: local=%d remote=%d\n", localTableId, remoteTableId);
			printf(pbuf);
#endif
			m_tableMap.insert(IDMap_t::value_type(remoteTableId, localTableId));
		}
		else if (value == kNetworkTables_ASSIGNMENT)
		{
			UINT32 tableId = input.ReadTableId(false);
			if (!m_connected)
				break;
			NetworkTable *table = GetTable(false, tableId);
			std::string keyName = input.ReadString();
			if (!m_connected)
				break;
			Key *key = table->GetKey(keyName.c_str());
			UINT32 id = input.ReadId(false);
			if (!m_connected)
				break;
#ifdef DEBUG
			char pbuf[64];
			snprintf(pbuf, 64, "Field Assignment: table %d \"%s\" local=%d remote=%d\n", tableId, keyName.c_str(), key->GetId(), id);
			printf(pbuf);
#endif
			m_fieldMap.insert(IDMap_t::value_type(id, key->GetId()));
		}
		else if (value == kNetworkTables_TRANSACTION)
		{
#ifdef DEBUG
			printf("Transaction Start\n");
#endif
			m_inTransaction = !m_inTransaction;
			// Finishing a transaction
			if (!m_inTransaction)
			{
				if (m_denyTransaction)
				{
					Offer(std::auto_ptr<Data>(new Denial(1)));
				}
				else
				{
					if (!m_transaction->IsEmpty())
						((Entry *)m_transaction->Peek())->GetKey()->GetTable()->ProcessTransaction(false, m_transaction);
					Offer(std::auto_ptr<Data>(new Confirmation(1)));
				}
				m_denyTransaction = false;
			}
#ifdef DEBUG
			printf("Transaction End\n");
#endif
		}
		else
		{
#ifdef DEBUG
			char buf[64];
			snprintf(buf, 64, "Don't know how to interpret marker byte (%02X)", value);
			wpi_setWPIErrorWithContext(NetworkTablesCorrupt, buf);
#else
			wpi_setWPIErrorWithContext(NetworkTablesCorrupt, "Don't know how to interpret marker byte");
#endif
			Close();
			return;
		}
		value = input.Read();
	}
}

void Connection::WriteTaskRun()
{
	std::auto_ptr<Buffer> buffer = std::auto_ptr<Buffer>(new Buffer(2048));
	bool sentData = true;
	while (m_connected)
	{
		std::pair<Data *, bool> data;
		{
			Synchronized sync(m_dataLock);
			data = m_queue->Poll();
			// Check if there is no data to send
			if (data.first == NULL)
			{
				// Ping if necessary
				if (sentData)
				{
					sentData = false;
				}
				else
				{
					buffer->WriteByte(kNetworkTables_PING);
					buffer->Flush(m_socket);
				}
				semGive(m_dataLock);
				semTake(m_dataAvailable, kWriteDelay);
				semTake(m_dataLock, WAIT_FOREVER);
				continue;
			}
		}

		// If there is data, send it
		sentData = true;

		if (data.first->IsEntry())
			m_confirmations.push_back((Entry *)data.first);
		else if (data.first->IsOldData())
			m_confirmations.push_back(((OldData *)data.first)->GetEntry());
		else if (data.first->IsTransaction())
			m_confirmations.push_back((Entry *)NULL);

		data.first->Encode(buffer.get());
		buffer->Flush(m_socket);
		// Noone else wants this data and it used to be auto_ptr'd, so delete it
		if (data.second)
			delete data.first;
	}
}

void Connection::Close()
{
	if (m_connected)
	{
		m_connected = false;
		close(m_socket);
		WatchdogFeed();
		IDMap_t::iterator it = m_tableMap.begin();
		IDMap_t::iterator end = m_tableMap.end();
		for (; it != end; it++)
		{
			// Get the local id
			UINT32 id = it->second;
			NetworkTable *table = NetworkTable::GetTable(id);
#ifdef DEBUG
			char pbuf[64];
			snprintf(pbuf, 64, "Removing Table %d (%p)\n", id, table);
			printf(pbuf);
#endif
			if (table)
				table->RemoveConnection(this);
		}

		ConnectionManager::GetInstance()->RemoveConnection(this);
	}
}

NetworkTable *Connection::GetTable(bool local, UINT32 id)
{
	NetworkTable *table = NULL;
	if (local)
	{
		table = NetworkTable::GetTable(id);
	}
	else
	{
		IDMap_t::iterator localID = m_tableMap.find(id);
		if (localID != m_tableMap.end())
		{
			table = NetworkTable::GetTable(localID->second);
		}
		/*
		else
		{
			// This should not be needed as long as TABLE_REQUEST is always issued first
			// We don't care about hosting locally anonymous tables from the network
			table = new NetworkTable();
			m_tableMap.insert(IDMap_t::value_type(id, table->GetId()));
			Offer(std::auto_ptr<Data>(new TableAssignment(table, id)));
			table->AddConnection(this);
		}
		*/
	}
	if (table == NULL)
	{
		wpi_setWPIErrorWithContext(NetworkTablesCorrupt, "Unexpected ID");
	}
	return table;
}

bool Connection::ConfirmationsContainsKey(Key *key)
{
	std::deque<Entry *>::iterator it = m_confirmations.begin();
	std::deque<Entry *>::iterator end = m_confirmations.end();
	for (; it != end; it++)
		if ((*it)->GetKey() == key)
			return true;

	return false;
}

void Connection::WatchdogTaskRun()
{
	Synchronized sync(m_watchdogLock);
	while (m_connected)
	{
		while(!m_watchdogActive)
		{
			semGive(m_watchdogLock);
			semTake(m_watchdogFood, WAIT_FOREVER);
			semTake(m_watchdogLock, WAIT_FOREVER);
		}
		m_watchdogFed = false;
		semGive(m_watchdogLock);
		int retval = semTake(m_watchdogFood, kTimeout);
		semTake(m_watchdogLock, WAIT_FOREVER);
		if (retval == ERROR && !m_watchdogFed)
		{
			wpi_setWPIErrorWithContext(Timeout, "NetworkTables watchdog expired... disconnecting");
			break;
		}
	}

	Close();
}

void Connection::WatchdogActivate()
{
	Synchronized sync(m_watchdogLock);
	if (!m_watchdogActive)
	{
		m_watchdogActive = true;
		semGive(m_watchdogFood);
	}
}

void Connection::WatchdogFeed()
{
	Synchronized sync(m_watchdogLock);
	m_watchdogActive = false;
	m_watchdogFed = true;
	semGive(m_watchdogFood);
}

} // namespace
