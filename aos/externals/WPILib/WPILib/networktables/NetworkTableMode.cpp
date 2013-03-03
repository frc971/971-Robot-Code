/*
 * NetworkTableMode.cpp
 *
 *  Created on: Oct 16, 2012
 *      Author: Mitchell Wills
 */

#include <string>
#include "networktables2/thread/NTThreadManager.h"
#include "networktables2/NetworkTableNode.h"
#include "networktables2/server/NetworkTableServer.h"
#include "networktables2/client/NetworkTableClient.h"
#include "networktables2/stream/SocketServerStreamProvider.h"
#include "networktables2/stream/SocketStreamFactory.h"
#include "networktables/NetworkTableMode.h"


NetworkTableServerMode NetworkTableMode::Server;
NetworkTableClientMode NetworkTableMode::Client;

NetworkTableServerMode::NetworkTableServerMode(){}
NetworkTableClientMode::NetworkTableClientMode(){}

NetworkTableNode* NetworkTableServerMode::CreateNode(const char* ipAddress, int port, NTThreadManager& threadManager){
	IOStreamProvider* streamProvider = new SocketServerStreamProvider(port);
	return new NetworkTableServer(*streamProvider, *new NetworkTableEntryTypeManager(), threadManager);
}
NetworkTableNode* NetworkTableClientMode::CreateNode(const char* ipAddress, int port, NTThreadManager& threadManager){
	IOStreamFactory* streamFactory = new SocketStreamFactory(ipAddress, port);
	return new NetworkTableClient(*streamFactory, *new NetworkTableEntryTypeManager(), threadManager);
}
