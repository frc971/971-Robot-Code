/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef ANALOG_MODULE_H_
#define ANALOG_MODULE_H_

#include "ChipObject.h"
#include "Module.h"

/**
 * Analog Module class.
 * Each module can independently sample its channels at a configurable rate.
 * There is a 64-bit hardware accumulator associated with channel 1 on each module.
 * The accumulator is attached to the output of the oversample and average engine so that the center
 * value can be specified in higher resolution resulting in less error.
 */
class AnalogModule: public Module
{ 
    friend class Module;

public:
	static const long kTimebase = 40000000; ///< 40 MHz clock
	static const long kDefaultOversampleBits = 0;
	static const long kDefaultAverageBits = 7;
	static const float kDefaultSampleRate = 50000.0;

	void SetSampleRate(float samplesPerSecond);
	float GetSampleRate();
	void SetAverageBits(UINT32 channel, UINT32 bits);
	UINT32 GetAverageBits(UINT32 channel);
	void SetOversampleBits(UINT32 channel, UINT32 bits);
	UINT32 GetOversampleBits(UINT32 channel);
	INT16 GetValue(UINT32 channel);
	INT32 GetAverageValue(UINT32 channel);
	float GetAverageVoltage(UINT32 channel);
	float GetVoltage(UINT32 channel);
	UINT32 GetLSBWeight(UINT32 channel);
	INT32 GetOffset(UINT32 channel);
	INT32 VoltsToValue(INT32 channel, float voltage);

	static AnalogModule* GetInstance(UINT8 moduleNumber);

protected:
	explicit AnalogModule(UINT8 moduleNumber);
	virtual ~AnalogModule();

private:
	static SEM_ID m_registerWindowSemaphore;

	UINT32 GetNumActiveChannels();
	UINT32 GetNumChannelsToActivate();
	void SetNumChannelsToActivate(UINT32 channels);

	tAI *m_module;
	bool m_sampleRateSet;
	UINT32 m_numChannelsToActivate;
};

#endif
