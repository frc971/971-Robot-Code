/*
 * NetworkTableServer.cpp
 *
 *  Created on: Sep 27, 2012
 *      Author: Mitchell Wills
 */

#include "networktables2/server/NetworkTableServer.h"
#include "networktables2/server/ServerNetworkTableEntryStore.h"
#include <iostream>
#include <limits.h>

NetworkTableServer::NetworkTableServer(IOStreamProvider& _streamProvider, NetworkTableEntryTypeManager& typeManager, NTThreadManager& threadManager):
		NetworkTableNode(*new ServerNetworkTableEntryStore(*this)),
		streamProvider(_streamProvider),
		connectionList(),
		writeManager(connectionList, threadManager, GetEntryStore(), ULONG_MAX),
		incomingStreamMonitor(streamProvider, (ServerNetworkTableEntryStore&)entryStore, *this, connectionList, typeManager, threadManager),
                continuingReceiver(writeManager){
	
	GetEntryStore().SetIncomingReceiver(&continuingReceiver);
	GetEntryStore().SetOutgoingReceiver(&continuingReceiver);
	
	incomingStreamMonitor.start();
	writeManager.start();
}
/*NetworkTableServer::NetworkTableServer(IOStreamProvider& streamProvider){
	this(streamProvider, new NetworkTableEntryTypeManager(), new DefaultThreadManager());
}*/
NetworkTableServer::~NetworkTableServer(){
	Close();
	delete &entryStore;
}

void NetworkTableServer::Close(){
	try{
		incomingStreamMonitor.stop();
		writeManager.stop();
		connectionList.closeAll();
		streamProvider.close();
	} catch (const std::exception& ex) {
	    //TODO print stack trace?
	}
}

void NetworkTableServer::OnNewConnection(ServerConnectionAdapter& connectionAdapter) {
	connectionList.add(connectionAdapter);
}


bool NetworkTableServer::IsConnected() {
	return true;
}


bool NetworkTableServer::IsServer() {
	return true;
}
