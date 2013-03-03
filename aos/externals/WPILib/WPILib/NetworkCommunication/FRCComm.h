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

#ifdef SIMULATION
#include <vxWorks_compat.h>
#define EXPORT_FUNC __declspec(dllexport) __cdecl
#else
#include <vxWorks.h>
#define EXPORT_FUNC
#endif

// Commandeer some bytes at the end for advanced I/O feedback.
#define IO_CONFIG_DATA_SIZE 32
#define SYS_STATUS_DATA_SIZE 44
#define USER_STATUS_DATA_SIZE (984 - IO_CONFIG_DATA_SIZE - SYS_STATUS_DATA_SIZE)
#define USER_DS_LCD_DATA_SIZE 128

struct FRCCommonControlData{
	UINT16 packetIndex;
	union {
		UINT8 control;
#ifdef SIMULATION
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
};

#define kFRC_NetworkCommunication_DynamicType_DSEnhancedIO_Input 17
#define kFRC_NetworkCommunication_DynamicType_DSEnhancedIO_Output 18
#define kFRC_NetworkCommunication_DynamicType_Kinect_Header 19
#define kFRC_NetworkCommunication_DynamicType_Kinect_Extra1 20
#define kFRC_NetworkCommunication_DynamicType_Kinect_Vertices1 21
#define kFRC_NetworkCommunication_DynamicType_Kinect_Extra2 22
#define kFRC_NetworkCommunication_DynamicType_Kinect_Vertices2 23
#define kFRC_NetworkCommunication_DynamicType_Kinect_Joystick 24
#define kFRC_NetworkCommunication_DynamicType_Kinect_Custom 25

extern "C" {
#ifndef SIMULATION
	void EXPORT_FUNC getFPGAHardwareVersion(UINT16 *fpgaVersion, UINT32 *fpgaRevision);
#endif
	int EXPORT_FUNC getCommonControlData(FRCCommonControlData *data, int wait_ms);
	int EXPORT_FUNC getRecentCommonControlData(FRCCommonControlData *commonData, int wait_ms);
	int EXPORT_FUNC getRecentStatusData(UINT8 *batteryInt, UINT8 *batteryDec, UINT8 *dsDigitalOut, int wait_ms);
	int EXPORT_FUNC getDynamicControlData(UINT8 type, char *dynamicData, INT32 maxLength, int wait_ms);
	int EXPORT_FUNC setStatusData(float battery, UINT8 dsDigitalOut, UINT8 updateNumber,
			const char *userDataHigh, int userDataHighLength,
			const char *userDataLow, int userDataLowLength, int wait_ms);
	int EXPORT_FUNC setStatusDataFloatAsInt(int battery, UINT8 dsDigitalOut, UINT8 updateNumber,
			const char *userDataHigh, int userDataHighLength,
			const char *userDataLow, int userDataLowLength, int wait_ms);
	int EXPORT_FUNC setErrorData(const char *errors, int errorsLength, int wait_ms);
	int EXPORT_FUNC setUserDsLcdData(const char *userDsLcdData, int userDsLcdDataLength, int wait_ms);
	int EXPORT_FUNC overrideIOConfig(const char *ioConfig, int wait_ms);

	void EXPORT_FUNC setNewDataSem(SEM_ID);
#ifndef SIMULATION
	void EXPORT_FUNC setResyncSem(SEM_ID);
	void EXPORT_FUNC signalResyncActionDone(void);
#endif

	// this UINT32 is really a LVRefNum
	void EXPORT_FUNC setNewDataOccurRef(UINT32 refnum);
#ifndef SIMULATION
	void EXPORT_FUNC setResyncOccurRef(UINT32 refnum);
#endif

	void EXPORT_FUNC FRC_NetworkCommunication_getVersionString(char *version);
	void EXPORT_FUNC FRC_NetworkCommunication_observeUserProgramStarting(void);
	void EXPORT_FUNC FRC_NetworkCommunication_observeUserProgramDisabled(void);
	void EXPORT_FUNC FRC_NetworkCommunication_observeUserProgramAutonomous(void);
	void EXPORT_FUNC FRC_NetworkCommunication_observeUserProgramTeleop(void);
	void EXPORT_FUNC FRC_NetworkCommunication_observeUserProgramTest(void);
};

#endif
