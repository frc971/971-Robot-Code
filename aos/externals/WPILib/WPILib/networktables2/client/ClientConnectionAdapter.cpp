/*
 * ClientConnectionAdapter.cpp
 *
 *  Created on: Nov 2, 2012
 *      Author: Mitchell Wills
 */

#include "networktables2/client/ClientConnectionAdapter.h"
#include "networktables2/connection/ConnectionMonitorThread.h"

void ClientConnectionAdapter::gotoState(ClientConnectionState* newState){
	{
		Synchronized sync(LOCK);
		if(connectionState!=newState){
			fprintf(stdout, "%p entered connection state: %s\n",this, newState->toString());
			fflush(stdout);
			if(newState==&ClientConnectionState::IN_SYNC_WITH_SERVER)
				connectionListenerManager.FireConnectedEvent();
			if(connectionState==&ClientConnectionState::IN_SYNC_WITH_SERVER)
				connectionListenerManager.FireDisconnectedEvent();
			connectionState = newState;
		}
	}
}
/**
 * @return the state of the connection
 */
ClientConnectionState* ClientConnectionAdapter::getConnectionState(){
	return connectionState;
}
/**
 * @return if the client is connected to the server
 */
bool ClientConnectionAdapter::isConnected() {
	return getConnectionState()==&ClientConnectionState::IN_SYNC_WITH_SERVER;
}

/**
 * Create a new ClientConnectionAdapter
 * @param entryStore
 * @param threadManager
 * @param streamFactory
 * @param transactionPool
 * @param connectionListenerManager
 */
ClientConnectionAdapter::ClientConnectionAdapter(ClientNetworkTableEntryStore& _entryStore, NTThreadManager& _threadManager, IOStreamFactory& _streamFactory, ClientConnectionListenerManager& _connectionListenerManager, NetworkTableEntryTypeManager& _typeManager):
	entryStore(_entryStore),
	streamFactory(_streamFactory),
	threadManager(_threadManager),
	connectionListenerManager(_connectionListenerManager),
	typeManager(_typeManager),
    readThread(NULL),
    connection(NULL){
	connectionState = &ClientConnectionState::DISCONNECTED_FROM_SERVER;
}
ClientConnectionAdapter::~ClientConnectionAdapter(){
  close();
}


/*
 * Connection management
 */
/**
 * Reconnect the client to the server (even if the client is not currently connected)
 */
void ClientConnectionAdapter::reconnect() {
	{
		Synchronized sync(LOCK);
		close();//close the existing stream and monitor thread if needed
		try{
			IOStream* stream = streamFactory.createStream();
			if(stream==NULL)
				return;
			connection = new NetworkTableConnection(stream, typeManager);
			readThread = threadManager.newBlockingPeriodicThread(new ConnectionMonitorThread(*this, *connection), "Client Connection Reader Thread");
			connection->sendClientHello();
			gotoState(&ClientConnectionState::CONNECTED_TO_SERVER);
		} catch(IOException& e){
			close();//make sure to clean everything up if we fail to connect
		}
	}
}

/**
 * Close the client connection
 */
void ClientConnectionAdapter::close() {
	close(&ClientConnectionState::DISCONNECTED_FROM_SERVER);
}
/**
 * Close the connection to the server and enter the given state
 * @param newState
 */
void ClientConnectionAdapter::close(ClientConnectionState* newState) {
	{
		Synchronized sync(LOCK);
		gotoState(newState);
		if(readThread!=NULL){
			readThread->stop();
			readThread = NULL;
		}
		if(connection!=NULL){
			connection->close();
			connection = NULL;
		}
		entryStore.clearIds();
	}
}



void ClientConnectionAdapter::badMessage(BadMessageException& e) {
	close(new ClientConnectionState_Error(e));
}

void ClientConnectionAdapter::ioException(IOException& e) {
	if(connectionState!=&ClientConnectionState::DISCONNECTED_FROM_SERVER)//will get io exception when on read thread connection is closed
		reconnect();
	//gotoState(new ClientConnectionState.Error(e));
}

NetworkTableEntry* ClientConnectionAdapter::GetEntry(EntryId id) {
	return entryStore.GetEntry(id);
}


void ClientConnectionAdapter::keepAlive() {
}

void ClientConnectionAdapter::clientHello(ProtocolVersion protocolRevision) {
	throw BadMessageException("A client should not receive a client hello message");
}

void ClientConnectionAdapter::protocolVersionUnsupported(ProtocolVersion protocolRevision) {
	close();
	gotoState(new ClientConnectionState_ProtocolUnsuppotedByServer(protocolRevision));
}

void ClientConnectionAdapter::serverHelloComplete() {
	if (connectionState==&ClientConnectionState::CONNECTED_TO_SERVER) {
		try {
			gotoState(&ClientConnectionState::IN_SYNC_WITH_SERVER);
			entryStore.sendUnknownEntries(*connection);
		} catch (IOException& e) {
			ioException(e);
		}
	}
	else
		throw BadMessageException("A client should only receive a server hello complete once and only after it has connected to the server");
}


void ClientConnectionAdapter::offerIncomingAssignment(NetworkTableEntry* entry) {
	entryStore.offerIncomingAssignment(entry);
}
void ClientConnectionAdapter::offerIncomingUpdate(NetworkTableEntry* entry, SequenceNumber sequenceNumber, EntryValue value) {
	entryStore.offerIncomingUpdate(entry, sequenceNumber, value);
}

void ClientConnectionAdapter::offerOutgoingAssignment(NetworkTableEntry* entry) {
	try {
		{
			Synchronized sync(LOCK);
			if(connection!=NULL && connectionState==&ClientConnectionState::IN_SYNC_WITH_SERVER)
				connection->sendEntryAssignment(*entry);
		}
	} catch(IOException& e){
		ioException(e);
	}
}

void ClientConnectionAdapter::offerOutgoingUpdate(NetworkTableEntry* entry) {
	try {
		{
			Synchronized sync(LOCK);
			if(connection!=NULL && connectionState==&ClientConnectionState::IN_SYNC_WITH_SERVER)
				connection->sendEntryUpdate(*entry);
		}
	} catch(IOException& e){
		ioException(e);
	}
}
void ClientConnectionAdapter::flush() {
	{
		Synchronized sync(LOCK);
		if(connection!=NULL) {
			try {
				connection->flush();
			} catch (IOException& e) {
				ioException(e);
			}
		}
	}
}
void ClientConnectionAdapter::ensureAlive() {
	{
		Synchronized sync(LOCK);
		if(connection!=NULL) {
			try {
			  connection->sendKeepAlive();
			} catch (IOException& e) {
				ioException(e);
			}
		}
		else
			reconnect();//try to reconnect if not connected
	}
}
