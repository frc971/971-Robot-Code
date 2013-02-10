/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __INTERFACE_CONSTANTS_H__
#define __INTERFACE_CONSTANTS_H__

#define kNetworkTables_STRING			0
#define kNetworkTables_BEGIN_STRING		0xFF
#define kNetworkTables_END_STRING		0
#define kNetworkTables_INT				1
#define kNetworkTables_DOUBLE			2
#define kNetworkTables_TABLE			3
#define kNetworkTables_TABLE_ASSIGNMENT	kNetworkTables_TABLE
#define kNetworkTables_BOOLEAN_FALSE	4
#define kNetworkTables_BOOLEAN_TRUE		5
#define kNetworkTables_ASSIGNMENT		6
#define kNetworkTables_EMPTY			7
#define kNetworkTables_DATA				8
#define kNetworkTables_OLD_DATA			9
#define kNetworkTables_TRANSACTION		10
#define kNetworkTables_REMOVAL			11
#define kNetworkTables_TABLE_REQUEST	12
#define kNetworkTables_ID				(1 << 7)
#define kNetworkTables_TABLE_ID			(1 << 6)
#define kNetworkTables_CONFIRMATION		(1 << 5)
#define kNetworkTables_CONFIRMATION_MAX	(kNetworkTables_CONFIRMATION - 1)
#define kNetworkTables_PING				kNetworkTables_CONFIRMATION
#define kNetworkTables_DENIAL			(1 << 4)

typedef enum
{
	kNetworkTables_Types_NONE = -1,
	kNetworkTables_Types_STRING = kNetworkTables_STRING,
	kNetworkTables_Types_INT = kNetworkTables_INT,
	kNetworkTables_Types_DOUBLE = kNetworkTables_DOUBLE,
	kNetworkTables_Types_BOOLEAN = kNetworkTables_BOOLEAN_TRUE,
	kNetworkTables_Types_TABLE = kNetworkTables_TABLE,
} NetworkTables_Types;

#endif // __INTERFACE_CONSTANTS_H__

