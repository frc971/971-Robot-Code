/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __NETWORK_QUEUE_H__
#define __NETWORK_QUEUE_H__

#include "NetworkTables/NetworkTable.h"
#include <map>
#include <deque>

namespace NetworkTables
{
class Entry;
class Confirmation;
class Denial;
class TransactionStart;
class TransactionEnd;
class Data;

class NetworkQueue
{
	typedef std::deque<std::pair<Data *, bool> > DataQueue_t;
	typedef std::map<UINT32, DataQueue_t::iterator> DataHash_t;
public:
	NetworkQueue();
	~NetworkQueue();

	void Offer(TransactionStart *value);
	void Offer(TransactionEnd *value);
	void Offer(Data *value, bool needsDelete=false);
	void Offer(std::auto_ptr<Data> value);
	bool IsEmpty();
	bool ContainsKey(Key *key);
	std::pair<Data *, bool> Poll();
	void Clear();
	Data *Peek();
	DataQueue_t::const_iterator GetQueueHead() {return m_dataQueue.begin();}
	bool IsQueueEnd(DataQueue_t::const_iterator it) {return m_dataQueue.end() == it;}

private:
	SEM_ID m_dataLock;
	DataQueue_t m_dataQueue;
	DataHash_t m_latestDataHash;
	bool m_inTransaction;
};

}

#endif // __NETWORK_QUEUE_H__
