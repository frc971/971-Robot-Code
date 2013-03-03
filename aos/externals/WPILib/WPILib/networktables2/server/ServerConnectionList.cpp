/*
 * ServerConnectionList.cpp
 *
 *  Created on: Sep 26, 2012
 *      Author: Mitchell Wills
 */

#include "networktables2/server/ServerConnectionList.h"
#include <algorithm>
#include <stdio.h>



ServerConnectionList::ServerConnectionList()
{
}
ServerConnectionList::~ServerConnectionList()
{
	connectionsLock.take();
	closeAll();
}

void ServerConnectionList::add(ServerConnectionAdapter& connection)
{ 
	Synchronized sync(connectionsLock);
	connections.push_back(&connection);
}

void ServerConnectionList::close(ServerConnectionAdapter& connectionAdapter, bool closeStream)
{ 
	Synchronized sync(connectionsLock);
	std::vector<ServerConnectionAdapter*>::iterator connectionPosition = std::find(connections.begin(), connections.end(), &connectionAdapter);
	if (connectionPosition != connections.end() && (*connectionPosition)==&connectionAdapter)
	{
		fprintf(stdout, "Close: %p\n", &connectionAdapter);
		fflush(stdout);
		connections.erase(connectionPosition);
		connectionAdapter.shutdown(closeStream);
		delete &connectionAdapter;
	}
}

void ServerConnectionList::closeAll()
{ 
	Synchronized sync(connectionsLock);
	while(connections.size() > 0)
	{
		close(*connections[0], true);
	}
}

void ServerConnectionList::offerOutgoingAssignment(NetworkTableEntry* entry)
{ 
	Synchronized sync(connectionsLock);
	for(unsigned int i = 0; i < connections.size(); ++i)
	{
		connections[i]->offerOutgoingAssignment(entry);
	}
}
void ServerConnectionList::offerOutgoingUpdate(NetworkTableEntry* entry)
{ 
	Synchronized sync(connectionsLock);
	for(unsigned int i = 0; i < connections.size(); ++i)
	{
		connections[i]->offerOutgoingUpdate(entry);
	}
}
void ServerConnectionList::flush()
{ 
	Synchronized sync(connectionsLock);
	for(unsigned int i = 0; i < connections.size(); ++i)
	{
		connections[i]->flush();
	}
}
void ServerConnectionList::ensureAlive()
{ 
	Synchronized sync(connectionsLock);
	for(unsigned int i = 0; i < connections.size(); ++i)
	{
		connections[i]->ensureAlive();
	}
}
