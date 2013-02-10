/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __CONNECTION_MANAGER_H__
#define __CONNECTION_MANAGER_H__

#include "NetworkTables/Data.h"
#include "SensorBase.h"
#include "Task.h"
#include <set>

namespace NetworkTables
{
class Connection;

class ConnectionManager : public SensorBase
{
	friend class Connection;
public:
	static ConnectionManager *GetInstance();

private:
	ConnectionManager();
	~ConnectionManager();

	int ListenTaskRun();
	bool IsServer() {return m_isServer;}
	void AddConnection(Connection *connection);
	void RemoveConnection(Connection *connection);

	static int InitListenTask(ConnectionManager *obj) {obj->ListenTaskRun();return 0;}

	typedef std::set<Connection *> ConnectionSet;
	bool m_isServer;
	ConnectionSet m_connections;
	Task m_listener;
	bool m_run;
	int m_listenSocket;
	SEM_ID m_connectionLock;

	static ConnectionManager *_instance;
};

}

#endif
