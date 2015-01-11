/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/
#pragma once

#include "SolenoidBase.h"

/**
 * Solenoid class for running high voltage Digital Output (PCM).
 * 
 * The Solenoid class is typically used for pneumatics solenoids, but could be used
 * for any device within the current spec of the PCM.
 */
class Solenoid : public SolenoidBase
{
public:
	explicit Solenoid(uint32_t channel);
	Solenoid(uint8_t moduleNumber, uint32_t channel);
	virtual ~Solenoid();
	virtual void Set(bool on);
	virtual bool Get();
	bool IsBlackListed();

private:
	void InitSolenoid();

	uint32_t m_channel; ///< The channel on the module to control.
};
