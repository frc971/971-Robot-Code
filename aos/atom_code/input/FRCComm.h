/*************************************************************
 * 					NOTICE
 * 
 * 	These are the only externally exposed functions to the
 *   NetworkCommunication library
 * 
 * This is an implementation of FRC Spec for Comm Protocol
 * Revision 4.5, June 30, 2008
 *
 * Copyright (c) National Instruments 2008.  All Rights Reserved.
 * 
 *************************************************************/

#ifndef __FRC_COMM_H__
#define __FRC_COMM_H__

#include <stdint.h>

typedef uint64_t UINT64;
typedef uint32_t UINT32;
typedef uint16_t UINT16;
typedef uint8_t  UINT8;
typedef int8_t  INT8;

struct FRCCommonControlData{
	UINT16 packetIndex;
	union {
		UINT8 control;
    // The order of the bits has to be flipped on little-endian machines (aka
    // everything other than the cRIO that we build for) in order for it to
    // work. Upstream WPILib does this based off of a different macro.
#ifndef __VXWORKS__
		struct {
			UINT8 checkVersions :1;
			UINT8 test :1;
			UINT8 resync : 1;
			UINT8 fmsAttached:1;
			UINT8 autonomous : 1;
			UINT8 enabled : 1;
			UINT8 notEStop : 1;
			UINT8 reset : 1;
		};
#else
		struct {
			UINT8 reset : 1;
			UINT8 notEStop : 1;
			UINT8 enabled : 1;
			UINT8 autonomous : 1;
			UINT8 fmsAttached:1;
			UINT8 resync : 1;
			UINT8 test :1;
			UINT8 checkVersions :1;
		};
#endif
	};
	UINT8 dsDigitalIn;
	UINT16 teamID;

	char dsID_Alliance;
	char dsID_Position;

	union {
		INT8 stick0Axes[6];
		struct {
			INT8 stick0Axis1;
			INT8 stick0Axis2;
			INT8 stick0Axis3;
			INT8 stick0Axis4;
			INT8 stick0Axis5;
			INT8 stick0Axis6;
		};
	};
	UINT16 stick0Buttons;		// Left-most 4 bits are unused

	union {
		INT8 stick1Axes[6];
		struct {
			INT8 stick1Axis1;
			INT8 stick1Axis2;
			INT8 stick1Axis3;
			INT8 stick1Axis4;
			INT8 stick1Axis5;
			INT8 stick1Axis6;
		};
	};
	UINT16 stick1Buttons;		// Left-most 4 bits are unused

	union {
		INT8 stick2Axes[6];
		struct {
			INT8 stick2Axis1;
			INT8 stick2Axis2;
			INT8 stick2Axis3;
			INT8 stick2Axis4;
			INT8 stick2Axis5;
			INT8 stick2Axis6;
		};
	};
	UINT16 stick2Buttons;		// Left-most 4 bits are unused

	union {
		INT8 stick3Axes[6];
		struct {
			INT8 stick3Axis1;
			INT8 stick3Axis2;
			INT8 stick3Axis3;
			INT8 stick3Axis4;
			INT8 stick3Axis5;
			INT8 stick3Axis6;
		};
	};
	UINT16 stick3Buttons;		// Left-most 4 bits are unused

	//Analog inputs are 10 bit right-justified
	UINT16 analog1;
	UINT16 analog2;
	UINT16 analog3;
	UINT16 analog4;

	UINT64 cRIOChecksum;
	UINT32 FPGAChecksum0;
	UINT32 FPGAChecksum1;
	UINT32 FPGAChecksum2;
	UINT32 FPGAChecksum3;

	char versionData[8];
} __attribute__((packed));

#endif
