/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef DIGITAL_OUTPUT_H_
#define DIGITAL_OUTPUT_H_

#include "DigitalSource.h"
#include "LiveWindow/LiveWindowSendable.h"
#include "tables/ITableListener.h"

class DigitalModule;

/**
 * Class to write to digital outputs.
 * Write values to the digital output channels. Other devices implemented elsewhere will allocate
 * channels automatically so for those devices it shouldn't be done here.
 */
class DigitalOutput : public DigitalSource, public ITableListener, public LiveWindowSendable
{
public:
	explicit DigitalOutput(UINT32 channel);
	DigitalOutput(UINT8 moduleNumber, UINT32 channel);
	virtual ~DigitalOutput();
	void Set(UINT32 value);
	UINT32 GetChannel();
	void Pulse(float length);
	bool IsPulsing();
	void SetPWMRate(float rate);
	void EnablePWM(float initialDutyCycle);
	void DisablePWM();
	void UpdateDutyCycle(float dutyCycle);

	// Digital Source Interface
	virtual UINT32 GetChannelForRouting();
	virtual UINT32 GetModuleForRouting();
	virtual bool GetAnalogTriggerForRouting();
	virtual void RequestInterrupts(tInterruptHandler handler, void *param);
	virtual void RequestInterrupts();

	void SetUpSourceEdge(bool risingEdge, bool fallingEdge);
	
	virtual void ValueChanged(ITable* source, const std::string& key, EntryValue value, bool isNew);
	void UpdateTable();
	void StartLiveWindowMode();
	void StopLiveWindowMode();
	std::string GetSmartDashboardType();
	void InitTable(ITable *subTable);
	ITable * GetTable();

private:
	void InitDigitalOutput(UINT8 moduleNumber, UINT32 channel);

	UINT32 m_channel;
	UINT32 m_pwmGenerator;
	DigitalModule *m_module;
	
	ITable *m_table;
};

#endif
