/**
 * An abstraction for the NetworkTable protocol
 * 
 * @author mwills
 *
 */


#ifndef NETWORK_TABLE_CONNECTION_H_
#define NETWORK_TABLE_CONNECTION_H_

#include <stdio.h>
#include <stdlib.h>
#include "Synchronized.h"
#ifndef _WRS_KERNEL
#include <stdint.h>
#endif

class NetworkTableConnection;
typedef uint16_t ProtocolVersion;

#include "DataIOStream.h"
#include "../NetworkTableEntry.h"
#include "../type/NetworkTableEntryType.h"
#include "../type/NetworkTableEntryTypeManager.h"
#include "ConnectionAdapter.h"
#include "../NetworkTableMessageType.h"


class NetworkTableConnection{
public:
	static const ProtocolVersion PROTOCOL_REVISION = 0x0200;

	NetworkTableConnection(IOStream* stream, NetworkTableEntryTypeManager& typeManager);
	~NetworkTableConnection();
	void close();
	void flush();
	void sendKeepAlive();
	void sendClientHello();
	void sendServerHelloComplete();
	void sendProtocolVersionUnsupported();
	void sendEntryAssignment(NetworkTableEntry& entry);
	void sendEntryUpdate(NetworkTableEntry& entry);
	void read(ConnectionAdapter& adapter);
private:
	ReentrantSemaphore WRITE_LOCK;
	DataIOStream* const ioStream;
	NetworkTableEntryTypeManager& typeManager;
	bool isValid;

	void sendMessageHeader(NetworkTableMessageType messageType);
};



#endif
