/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __DRIVER_STATION_ENHANCED_IO_H__
#define __DRIVER_STATION_ENHANCED_IO_H__

#include "ErrorBase.h"
#include "NetworkCommunication/FRCComm.h"
#include <stack>
#include <vector>
#include <vxWorks.h>

#define kAnalogInputResolution ((double)((1<<14)-1))
#define kAnalogInputReference 3.3
#define kAnalogOutputResolution ((double)((1<<8)-1))
#define kAnalogOutputReference 4.0
#define kAccelOffset 8300
#define kAccelScale 3300.0
#define kSupportedAPIVersion 1

/**
 * Interact with the more complete I/O available from the 
 * newest driver station.  Get a reference to an object
 * of this type by calling GetEnhancedIO() on the DriverStation object.
 */
class DriverStationEnhancedIO : public ErrorBase
{
	// Can only be constructed by the DriverStation class.
	friend class DriverStation;

#pragma pack(push,1)
	// BEGIN: Definitions from the Cypress firmware
	typedef struct
	{
		UINT16 digital;
		UINT16 digital_oe;
		UINT16 digital_pe;
		UINT16 pwm_compare[4];
		UINT16 pwm_period[2];
		UINT8 dac[2];
		UINT8 leds;
		union
		{
			struct
			{
				// Bits are inverted from cypress fw because of big-endian!
				UINT8 pwm_enable : 4;
				UINT8 comparator_enable : 2;
				UINT8 quad_index_enable : 2;
			};
			UINT8 enables;
		};
		UINT8 fixed_digital_out;
	} output_t;  //data to IO (23 bytes)

	typedef struct
	{
		UINT8 api_version;
		UINT8 fw_version;
		INT16 analog[8];
		UINT16 digital;
		INT16 accel[3];
		INT16 quad[2];
		UINT8 buttons;
		UINT8 capsense_slider;
		UINT8 capsense_proximity;
	} input_t;	//data from IO (33 bytes)
	// END: Definitions from the Cypress firmware

	// Dynamic block definitions
	typedef struct
	{
		UINT8 size; // Must be 25 (size remaining in the block not counting the size variable)
		UINT8 id; // Must be 18
		output_t data;
		UINT8 flags;
	} status_block_t;

	typedef struct
	{
		UINT8 size; // Must be 34
		UINT8 id; // Must be 17
		input_t data;
	} control_block_t;
#pragma pack(pop)

	enum tBlockID
	{
		kInputBlockID = kFRC_NetworkCommunication_DynamicType_DSEnhancedIO_Input,
		kOutputBlockID = kFRC_NetworkCommunication_DynamicType_DSEnhancedIO_Output,
	};
	enum tStatusFlags {kStatusValid = 0x01, kStatusConfigChanged = 0x02, kForceEnhancedMode = 0x04};

public:
	enum tDigitalConfig {kUnknown, kInputFloating, kInputPullUp, kInputPullDown, kOutput, kPWM, kAnalogComparator};
	enum tAccelChannel {kAccelX = 0, kAccelY = 1, kAccelZ = 2};
	enum tPWMPeriodChannels {kPWMChannels1and2, kPWMChannels3and4};

	double GetAcceleration(tAccelChannel channel);
	double GetAnalogIn(UINT32 channel);
	double GetAnalogInRatio(UINT32 channel);
	double GetAnalogOut(UINT32 channel);
	void SetAnalogOut(UINT32 channel, double value);
	bool GetButton(UINT32 channel);
	UINT8 GetButtons();
	void SetLED(UINT32 channel, bool value);
	void SetLEDs(UINT8 value);
	bool GetDigital(UINT32 channel);
	UINT16 GetDigitals();
	void SetDigitalOutput(UINT32 channel, bool value);
	tDigitalConfig GetDigitalConfig(UINT32 channel);
	void SetDigitalConfig(UINT32 channel, tDigitalConfig config);
	double GetPWMPeriod(tPWMPeriodChannels channels);
	void SetPWMPeriod(tPWMPeriodChannels channels, double period);
	bool GetFixedDigitalOutput(UINT32 channel);
	void SetFixedDigitalOutput(UINT32 channel, bool value);
	INT16 GetEncoder(UINT32 encoderNumber);
	void ResetEncoder(UINT32 encoderNumber);
	bool GetEncoderIndexEnable(UINT32 encoderNumber);
	void SetEncoderIndexEnable(UINT32 encoderNumber, bool enable);
	double GetTouchSlider();
	double GetPWMOutput(UINT32 channel);
	void SetPWMOutput(UINT32 channel, double value);
	UINT8 GetFirmwareVersion();

private:
	DriverStationEnhancedIO();
	virtual ~DriverStationEnhancedIO();
	void UpdateData();
	void MergeConfigIntoOutput(const status_block_t &dsOutputBlock, status_block_t &localCache);
	bool IsConfigEqual(const status_block_t &dsOutputBlock, const status_block_t &localCache);

	// Usage Guidelines...
	DISALLOW_COPY_AND_ASSIGN(DriverStationEnhancedIO);

	control_block_t m_inputData;
	status_block_t m_outputData;
	SEM_ID m_inputDataSemaphore;
	SEM_ID m_outputDataSemaphore;
	bool m_inputValid;
	bool m_outputValid;
	bool m_configChanged;
	bool m_requestEnhancedEnable;
	INT16 m_encoderOffsets[2];
};

#endif

