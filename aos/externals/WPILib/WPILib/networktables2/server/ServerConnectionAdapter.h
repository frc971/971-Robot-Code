/*
 * ServerConnectionAdapter.h
 *
 *  Created on: Sep 26, 2012
 *      Author: Mitchell Wills
 */

#ifndef SERVERCONNECTIONADAPTER_H_
#define SERVERCONNECTIONADAPTER_H_

class ServerConnectionAdapter;

#include "networktables2/connection/ConnectionMonitorThread.h"
#include "networktables2/NetworkTableEntry.h"
#include "networktables2/connection/ConnectionAdapter.h"
#include "networktables2/stream/IOStream.h"
#include "networktables2/IncomingEntryReceiver.h"
#include "networktables2/FlushableOutgoingEntryReceiver.h"
#include "networktables2/server/ServerNetworkTableEntryStore.h"
#include "networktables2/server/ServerAdapterManager.h"
#include "networktables2/server/ServerConnectionState.h"
#include "networktables2/thread/NTThread.h"
#include "networktables2/thread/NTThreadManager.h"



/**
 * Object that adapts messages from a client to the server
 * 
 * @author Mitchell
 *
 */
class ServerConnectionAdapter : ConnectionAdapter, IncomingEntryReceiver, FlushableOutgoingEntryReceiver{
private:
	ServerNetworkTableEntryStore& entryStore;
	IncomingEntryReceiver& transactionReceiver;
	ServerAdapterManager& adapterListener;
public:
	/**
	 * the connection this adapter uses
	 */
	NetworkTableConnection connection;
private:
	NTThread* readThread;
	ConnectionMonitorThread monitorThread;
private:

	ServerConnectionState* connectionState;

	void gotoState(ServerConnectionState* newState);

public:
	/**
	 * Create a server connection adapter for a given stream
	 * 
	 * @param stream
	 * @param transactionPool
	 * @param entryStore
	 * @param transactionReceiver
	 * @param adapterListener
	 * @param threadManager
	 */
	ServerConnectionAdapter(IOStream* stream, ServerNetworkTableEntryStore& entryStore, IncomingEntryReceiver& transactionReceiver, ServerAdapterManager& adapterListener, NetworkTableEntryTypeManager& typeManager, NTThreadManager& threadManager);
	virtual ~ServerConnectionAdapter();

	void badMessage(BadMessageException& e);
	
	void ioException(IOException& e);
	
	
	/**
	 * stop the read thread and close the stream
	 */
	void shutdown(bool closeStream);

	void keepAlive();

	void clientHello(ProtocolVersion protocolRevision);

	void protocolVersionUnsupported(ProtocolVersion protocolRevision);

	void serverHelloComplete();

	void offerIncomingAssignment(NetworkTableEntry* entry);

	void offerIncomingUpdate(NetworkTableEntry* entry, SequenceNumber sequenceNumber, EntryValue value);

	NetworkTableEntry* GetEntry(EntryId id);

	void offerOutgoingAssignment(NetworkTableEntry* entry);
	
	void offerOutgoingUpdate(NetworkTableEntry* entry);


	void flush();

	/**
	 * @return the state of the connection
	 */
	ServerConnectionState* getConnectionState();

	void ensureAlive();

};


#endif /* SERVERCONNECTIONADAPTER_H_ */
