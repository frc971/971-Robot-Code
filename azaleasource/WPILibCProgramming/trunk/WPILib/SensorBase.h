/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef SENSORBASE_H_
#define SENSORBASE_H_

#include "ChipObject/NiRio.h"
#include "ErrorBase.h"
#include <stdio.h>
#include "Base.h"

/**
 * Base class for all sensors.
 * Stores most recent status information as well as containing utility functions for checking
 * channels and error processing.
 */
class SensorBase: public ErrorBase {
public:
	SensorBase();
	virtual ~SensorBase();
	static void DeleteSingletons();
	static UINT32 GetDefaultAnalogModule() { return 1; }
	static UINT32 GetDefaultDigitalModule() { return 1; }
	static UINT32 GetDefaultSolenoidModule() { return 1; }
	static bool CheckAnalogModule(UINT8 moduleNumber);
	static bool CheckDigitalModule(UINT8 moduleNumber);
	static bool CheckPWMModule(UINT8 moduleNumber);
	static bool CheckRelayModule(UINT8 moduleNumber);
	static bool CheckSolenoidModule(UINT8 moduleNumber);
	static bool CheckDigitalChannel(UINT32 channel);
	static bool CheckRelayChannel(UINT32 channel);
	static bool CheckPWMChannel(UINT32 channel);
	static bool CheckAnalogChannel(UINT32 channel);
	static bool CheckSolenoidChannel(UINT32 channel);

	static const UINT32 kSystemClockTicksPerMicrosecond = 40;
	static const UINT32 kDigitalChannels = 14;
	static const UINT32 kAnalogChannels = 8;
	static const UINT32 kAnalogModules = 2;
	static const UINT32 kDigitalModules = 2;
	static const UINT32 kSolenoidChannels = 8;
	static const UINT32 kSolenoidModules = 2;
	static const UINT32 kPwmChannels = 10;
	static const UINT32 kRelayChannels = 8;
	static const UINT32 kChassisSlots = 8;
protected:
	void AddToSingletonList();

private:
	DISALLOW_COPY_AND_ASSIGN(SensorBase);
	static SensorBase *m_singletonList;
	SensorBase *m_nextSingleton;
};


#endif
