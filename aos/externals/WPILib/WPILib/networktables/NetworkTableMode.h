#ifndef _NETWORKTABLEMODE_H_
#define _NETWORKTABLEMODE_H_


class NetworkTableMode;
class NetworkTableServerMode;
class NetworkTableClientMode;

#include <string>
#include "networktables2/NetworkTableNode.h"
#include "networktables2/thread/NTThreadManager.h"


/**
 * 
 * Represents a different modes that network tables can be configured in
 * 
 * @author Mitchell
 *
 */
class NetworkTableMode {
public:
	
	/**
	 * @param ipAddress the IP address configured by the user
	 * @param port the port configured by the user
	 * @param threadManager the thread manager that should be used for threads in the node
	 * @return a new node that can back a network table
	 * @throws IOException
	 */
	virtual NetworkTableNode* CreateNode(const char* ipAddress, int port, NTThreadManager& threadManager) = 0;
	
	static NetworkTableServerMode Server;
	static NetworkTableClientMode Client;
};

class NetworkTableServerMode : public NetworkTableMode{
public:
	NetworkTableServerMode();
	virtual NetworkTableNode* CreateNode(const char* ipAddress, int port, NTThreadManager& threadManager);
};

class NetworkTableClientMode : public NetworkTableMode{
public:
	NetworkTableClientMode();
	virtual NetworkTableNode* CreateNode(const char* ipAddress, int port, NTThreadManager& threadManager);
};


#endif
