/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef DIGITAL_INPUT_H_
#define DIGITAL_INPUT_H_

class DigitalModule;

#include "DigitalSource.h"
#include "LiveWindow/LiveWindowSendable.h"

/**
 * Class to read a digital input.
 * This class will read digital inputs and return the current value on the channel. Other devices
 * such as encoders, gear tooth sensors, etc. that are implemented elsewhere will automatically
 * allocate digital inputs and outputs as required. This class is only for devices like switches
 * etc. that aren't implemented anywhere else.
 */
class DigitalInput : public DigitalSource, public LiveWindowSendable {
public:
	explicit DigitalInput(UINT32 channel);
	DigitalInput(UINT8 moduleNumber, UINT32 channel);
	virtual ~DigitalInput();
	UINT32 Get();
	UINT32 GetChannel();

	// Digital Source Interface
	virtual UINT32 GetChannelForRouting();
	virtual UINT32 GetModuleForRouting();
	virtual bool GetAnalogTriggerForRouting();
	
	// Interruptable Interface
	virtual void RequestInterrupts(tInterruptHandler handler, void *param=NULL); ///< Asynchronus handler version.
	virtual void RequestInterrupts();		///< Synchronus Wait version.
	void SetUpSourceEdge(bool risingEdge, bool fallingEdge);

	void UpdateTable();
	void StartLiveWindowMode();
	void StopLiveWindowMode();
	std::string GetSmartDashboardType();
	void InitTable(ITable *subTable);
	ITable * GetTable();

private:
	void InitDigitalInput(UINT8 moduleNumber, UINT32 channel);
	UINT32 m_channel;
	DigitalModule *m_module;
	bool m_lastValue;
	
	ITable *m_table;
};

#endif

