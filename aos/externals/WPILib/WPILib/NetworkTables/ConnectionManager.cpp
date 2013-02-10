/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "NetworkTables/ConnectionManager.h"

#include "NetworkTables/Connection.h"
#include "NetworkTables/InterfaceConstants.h"
#include "Synchronized.h"
#include "WPIErrors.h"

#include <inetLib.h>
#include <selectLib.h>
#include <semLib.h>
#include <sockLib.h>
#include <taskLib.h>
#include <usrLib.h>

#define kPort 1735

namespace NetworkTables
{

ConnectionManager *ConnectionManager::_instance = NULL;

ConnectionManager::ConnectionManager() :
	m_isServer(true),
	m_listener("NetworkTablesListener", (FUNCPTR)InitListenTask),
	m_run(true),
	m_listenSocket(-1),
	m_connectionLock(NULL)
{
	AddToSingletonList();
	m_connectionLock = semMCreate(SEM_Q_PRIORITY | SEM_INVERSION_SAFE | SEM_DELETE_SAFE);
	m_listener.Start((UINT32)this);
}

ConnectionManager::~ConnectionManager()
{
	close(m_listenSocket);
	_instance->m_run = false;
	while(_instance->m_listener.Verify())
		taskDelay(10);
	semTake(m_connectionLock, WAIT_FOREVER);
	m_connections.clear();
	semDelete(m_connectionLock);
}

ConnectionManager *ConnectionManager::GetInstance()
{
	if (_instance == NULL)
		_instance = new ConnectionManager();
	return _instance;
}

int ConnectionManager::ListenTaskRun()
{
	struct sockaddr_in serverAddr;
	int sockAddrSize = sizeof(serverAddr);
	bzero((char *)&serverAddr, sockAddrSize);
	serverAddr.sin_len = (u_char)sockAddrSize;
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(kPort);
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

	// Create the socket.
	if ((m_listenSocket = socket(AF_INET, SOCK_STREAM, 0)) == ERROR)
	{
		printErrno(0);
		wpi_setGlobalWPIErrorWithContext(ResourceAlreadyAllocated, "Could not create NetworkTables server socket");
		return -1;
	}

	// Set the TCP socket so that it can be reused if it is in the wait state.
	int reuseAddr = 1;
	setsockopt(m_listenSocket, SOL_SOCKET, SO_REUSEADDR, (char *)&reuseAddr, sizeof(reuseAddr));

	// Bind socket to local address.
	if (bind(m_listenSocket, (struct sockaddr *)&serverAddr, sockAddrSize) == ERROR)
	{
		close(m_listenSocket);
		printErrno(0);
		wpi_setGlobalWPIErrorWithContext(ResourceAlreadyAllocated, "Could not bind NetworkTables server socket");
		return -1;
	}

	if (listen(m_listenSocket, 1) == ERROR)
	{
		close(m_listenSocket);
		printErrno(0);
		wpi_setGlobalWPIErrorWithContext(ResourceAlreadyAllocated, "Could not listen on NetworkTables server socket");
		return -1;
	}

	struct timeval timeout;
	// Check for a shutdown once per second
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;
	while (m_run)
	{
		fd_set fdSet;

		FD_ZERO(&fdSet);
		FD_SET(m_listenSocket, &fdSet);
		if (select(FD_SETSIZE, &fdSet, NULL, NULL, &timeout) > 0)
		{
			if (FD_ISSET(m_listenSocket, &fdSet))
			{
				struct sockaddr clientAddr;
				int clientAddrSize;
				int connectedSocket = accept(m_listenSocket, &clientAddr, &clientAddrSize);
				if (connectedSocket == ERROR)
					continue;

				// TODO: Linger option?
				AddConnection(new Connection(connectedSocket));
			}
		}
	}
	return 0;
}

void ConnectionManager::AddConnection(Connection *connection)
{
	{
		Synchronized sync(m_connectionLock);
		if (!m_connections.insert(connection).second)
		{
			wpi_setGlobalWPIErrorWithContext(ResourceAlreadyAllocated, "Connection object already exists");
			return;
		}
	}
	connection->Start();
}

void ConnectionManager::RemoveConnection(Connection *connection)
{
	{
		Synchronized sync(m_connectionLock);
		m_connections.erase(connection);
	}
	delete connection;
}

}
