/*
 * ConnectionMonitorThread.cpp
 *
 *  Created on: Sep 22, 2012
 *      Author: Mitchell Wills
 */

#include "networktables2/connection/ConnectionMonitorThread.h"
#include "networktables2/connection/BadMessageException.h"


ConnectionMonitorThread::ConnectionMonitorThread(ConnectionAdapter& _adapter, NetworkTableConnection& _connection) :
	adapter(_adapter), connection(_connection) {
}

void ConnectionMonitorThread::run() {
  
	try{
		connection.read(adapter);
	} catch(BadMessageException& e){
		adapter.badMessage(e);
	} catch(IOException& e){
		adapter.ioException(e);
	}
}

