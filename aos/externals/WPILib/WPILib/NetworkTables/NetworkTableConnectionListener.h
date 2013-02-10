/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __NETWORK_TABLE_CONNECTION_LISTENER_H__
#define __NETWORK_TABLE_CONNECTION_LISTENER_H__

class NetworkTable;

class NetworkTableConnectionListener {
public:
	virtual void Connected(NetworkTable *table) = 0;
	virtual void Disconnected(NetworkTable *table) = 0;
};

#endif // __NETWORK_TABLE_CONNECTION_LISTENER_H__
