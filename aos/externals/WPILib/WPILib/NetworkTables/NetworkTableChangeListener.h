/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __NETWORK_TABLE_CHANGE_LISTENER_H__
#define __NETWORK_TABLE_CHANGE_LISTENER_H__

#include "NetworkTables/InterfaceConstants.h"

class NetworkTable;

class NetworkTableChangeListener {
public:
public:
	virtual void ValueChanged(NetworkTable *table, const char *name, NetworkTables_Types type) = 0;
	virtual void ValueConfirmed(NetworkTable *table, const char *name, NetworkTables_Types type) = 0;
};

#endif // __NETWORK_TABLE_CHANGE_LISTENER_H__
