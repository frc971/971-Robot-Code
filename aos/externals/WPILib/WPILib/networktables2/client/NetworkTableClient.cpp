/*
 * NetworkTableClient.cpp
 *
 *  Created on: Nov 3, 2012
 *      Author: Mitchell Wills
 */

#include "networktables2/client/NetworkTableClient.h"
#include "networktables2/TransactionDirtier.h"

/**
 * Create a new NetworkTable Client
 * @param streamFactory
 * @param threadManager
 * @param transactionPool
 */
NetworkTableClient::NetworkTableClient(IOStreamFactory& streamFactory, NetworkTableEntryTypeManager& typeManager, NTThreadManager& threadManager):
	NetworkTableNode(*new ClientNetworkTableEntryStore(*this)),
	adapter((ClientNetworkTableEntryStore&)entryStore, threadManager, streamFactory, *this, typeManager),
	writeManager(adapter, threadManager, GetEntryStore(), 1000){
	
	GetEntryStore().SetOutgoingReceiver(new TransactionDirtier(writeManager));
	GetEntryStore().SetIncomingReceiver(&OutgoingEntryReceiver_NULL);
	writeManager.start();
}
NetworkTableClient::~NetworkTableClient(){}

/**
 * force the client to disconnect and reconnect to the server again. Will connect if the client is currently disconnected
 */
void NetworkTableClient::reconnect() {
	adapter.reconnect();
}

void NetworkTableClient::Close() {
	adapter.close();
}

void NetworkTableClient::stop() {
	writeManager.stop();
	Close();
}

bool NetworkTableClient::IsConnected() {
	return adapter.isConnected();
}

bool NetworkTableClient::IsServer() {
	return false;
}
