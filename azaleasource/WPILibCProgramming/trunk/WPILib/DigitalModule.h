/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef DIGITAL_MODULE_H_
#define DIGITAL_MODULE_H_

#include "Module.h"
#include "ChipObject.h"

class I2C;

const UINT32 kExpectedLoopTiming = 260;

class DigitalModule: public Module
{
	friend class I2C;
	friend class Module;

protected:
	explicit DigitalModule(UINT8 moduleNumber);
	virtual ~DigitalModule();

public:
	void SetPWM(UINT32 channel, UINT8 value);
	UINT8 GetPWM(UINT32 channel);
	void SetPWMPeriodScale(UINT32 channel, UINT32 squelchMask);
	void SetRelayForward(UINT32 channel, bool on);
	void SetRelayReverse(UINT32 channel, bool on);
	bool GetRelayForward(UINT32 channel);
	UINT8 GetRelayForward();
	bool GetRelayReverse(UINT32 channel);
	UINT8 GetRelayReverse();
	bool AllocateDIO(UINT32 channel, bool input);
	void FreeDIO(UINT32 channel);
	void SetDIO(UINT32 channel, short value);
	bool GetDIO(UINT32 channel);
	UINT16 GetDIO();
	bool GetDIODirection(UINT32 channel);
	UINT16 GetDIODirection();
	void Pulse(UINT32 channel, float pulseLength);
	bool IsPulsing(UINT32 channel);
	bool IsPulsing();
	UINT32 AllocateDO_PWM();
	void FreeDO_PWM(UINT32 pwmGenerator);
	void SetDO_PWMRate(float rate);
	void SetDO_PWMDutyCycle(UINT32 pwmGenerator, float dutyCycle);
	void SetDO_PWMOutputChannel(UINT32 pwmGenerator, UINT32 channel);

	I2C* GetI2C(UINT32 address);

	static DigitalModule* GetInstance(UINT8 moduleNumber);
	static UINT8 RemapDigitalChannel(UINT32 channel) { return 15 - channel; }; // TODO: Need channel validation
	static UINT8 UnmapDigitalChannel(UINT32 channel) { return 15 - channel; }; // TODO: Need channel validation

private:
	SEM_ID m_digitalSemaphore;
	SEM_ID m_relaySemaphore;
	SEM_ID m_doPwmSemaphore;
	tDIO *m_fpgaDIO;
};

#endif

