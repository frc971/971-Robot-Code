/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef ANALOG_TRIGGER_H_
#define ANALOG_TRIGGER_H_

#include "AnalogTriggerOutput.h"
#include "SensorBase.h"

class AnalogChannel;
class AnalogModule;

class AnalogTrigger: public SensorBase
{
	friend class AnalogTriggerOutput;
public:
	AnalogTrigger(UINT8 moduleNumber, UINT32 channel);
	explicit AnalogTrigger(UINT32 channel);
	explicit AnalogTrigger(AnalogChannel *channel);
	virtual ~AnalogTrigger();

	void SetLimitsVoltage(float lower, float upper);
	void SetLimitsRaw(INT32 lower, INT32 upper);
	void SetAveraged(bool useAveragedValue);
	void SetFiltered(bool useFilteredValue);
	UINT32 GetIndex();
	bool GetInWindow();
	bool GetTriggerState();
	AnalogTriggerOutput *CreateOutput(AnalogTriggerOutput::Type type);

private:
	void InitTrigger(UINT8 moduleNumber, UINT32 channel);

	UINT8 m_index;
	tAnalogTrigger *m_trigger;
	AnalogModule *m_analogModule;
	UINT32 m_channel;
};

#endif

