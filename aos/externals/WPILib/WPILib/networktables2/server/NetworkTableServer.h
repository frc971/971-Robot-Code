/*
 * NetworkTableServer.h
 *
 *  Created on: Sep 27, 2012
 *      Author: Mitchell Wills
 */

#ifndef NETWORKTABLESERVER_H_
#define NETWORKTABLESERVER_H_


class NetworkTableServer;

#include "networktables2/TransactionDirtier.h"
#include "networktables2/NetworkTableNode.h"
#include "ServerIncomingStreamMonitor.h"
#include "ServerIncomingConnectionListener.h"
#include "networktables2/WriteManager.h"
#include "networktables2/stream/IOStreamProvider.h"
#include "ServerConnectionList.h"

/**
 * A server node in NetworkTables 2.0
 * 
 * @author Mitchell
 *
 */
class NetworkTableServer : public NetworkTableNode, public ServerIncomingConnectionListener{
private:
	IOStreamProvider& streamProvider;
	ServerConnectionList connectionList;
	WriteManager writeManager;
	ServerIncomingStreamMonitor incomingStreamMonitor;
	TransactionDirtier continuingReceiver;

  public:
	/**
	 * Create a NetworkTable Server
	 * 
	 * @param streamProvider
	 * @param threadManager
	 * @param transactionPool
	 */
	NetworkTableServer(IOStreamProvider& streamProvider, NetworkTableEntryTypeManager& typeManager, NTThreadManager& threadManager);
	~NetworkTableServer();
	/**
	 * Create a NetworkTable Server
	 * 
	 * @param streamProvider
	 */
	NetworkTableServer(IOStreamProvider& streamProvider);
	
	void Close();

	void OnNewConnection(ServerConnectionAdapter& connectionAdapter);

	
	bool IsConnected();

	
	bool IsServer();

};



#endif /* NETWORKTABLESERVER_H_ */
