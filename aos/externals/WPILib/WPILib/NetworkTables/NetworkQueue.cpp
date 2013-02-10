/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "NetworkTables/NetworkQueue.h"

#include "NetworkTables/Entry.h"
#include "NetworkTables/Key.h"
#include "NetworkTables/Confirmation.h"
#include "NetworkTables/Denial.h"
#include "NetworkTables/TransactionStart.h"
#include "NetworkTables/TransactionEnd.h"
#include "Synchronized.h"

namespace NetworkTables
{

NetworkQueue::NetworkQueue() :
	m_dataLock(NULL),
	m_inTransaction(false)
{
	m_dataLock = semMCreate(SEM_Q_PRIORITY | SEM_INVERSION_SAFE | SEM_DELETE_SAFE);
}

NetworkQueue::~NetworkQueue()
{
	semDelete(m_dataLock);
}

void NetworkQueue::Offer(TransactionStart *value)
{
	Synchronized sync(m_dataLock);
	m_inTransaction = true;
	m_dataQueue.push_back(DataQueue_t::value_type(value, false));	
}

void NetworkQueue::Offer(TransactionEnd *value)
{
	Synchronized sync(m_dataLock);
	m_inTransaction = false;
	m_dataQueue.push_back(DataQueue_t::value_type(value, false));	
}

void NetworkQueue::Offer(Data *value, bool needsDelete)
{
	Synchronized sync(m_dataLock);
	if (value->IsEntry())
	{
		if (m_inTransaction)
		{
			m_dataQueue.push_back(DataQueue_t::value_type(value, needsDelete));
		}
		else
		{
			DataHash_t::iterator found = m_latestDataHash.find(((Entry *)value)->GetId());
			if (found != m_latestDataHash.end())
			{
				// Replace the old value for this key with a new one
				// If this used to be auto_ptr'd, then delete it
				// (This should never happen, right?)
				if (found->second->second)
					delete found->second->first;
				found->second->first = value;
			}
			else
			{
				// Add a new entry to the queue
				m_dataQueue.push_back(DataQueue_t::value_type(value, needsDelete));
				m_latestDataHash.insert(DataHash_t::value_type(((Entry *)m_dataQueue.back().first)->GetId(), m_dataQueue.end() - 1));
			}
		}
	}
	else
	{
		// TODO: Combine Confirmations
		// TODO: Combine Denials
		m_dataQueue.push_back(DataQueue_t::value_type(value, needsDelete));
	}
}

void NetworkQueue::Offer(std::auto_ptr<Data> value)
{
	// Indicate that we released this from an auto_ptr
	Offer(value.release(), true);
}

bool NetworkQueue::IsEmpty()
{
	Synchronized sync(m_dataLock);
	return m_dataQueue.empty();
}

bool NetworkQueue::ContainsKey(Key *key)
{
	Synchronized sync(m_dataLock);
	DataHash_t::iterator found = m_latestDataHash.find(key->GetId());
	return key != NULL && found != m_latestDataHash.end();
}

// @return the data and if it came from an auto_ptr
std::pair<Data *, bool> NetworkQueue::Poll()
{
	Synchronized sync(m_dataLock);
	if (IsEmpty())
		return DataQueue_t::value_type(NULL, false);
	DataQueue_t::value_type data = m_dataQueue.front();
	if (data.first->IsEntry())
	{
		m_latestDataHash.erase(((Entry *)data.first)->GetId());
	}
	m_dataQueue.pop_front();
	return data;
}

void NetworkQueue::Clear()
{
	Synchronized sync(m_dataLock);
	m_dataQueue.clear();
	m_latestDataHash.clear();
}

Data *NetworkQueue::Peek()
{
	Synchronized sync(m_dataLock);
	if (IsEmpty())
		return NULL;
	return m_dataQueue.front().first;
}

} // namespace

