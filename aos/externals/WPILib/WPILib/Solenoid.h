/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef SOLENOID_H_
#define SOLENOID_H_

#include "SolenoidBase.h"

/**
 * Solenoid class for running high voltage Digital Output (9472 module).
 * 
 * The Solenoid class is typically used for pneumatics solenoids, but could be used
 * for any device within the current spec of the 9472 module.
 */
class Solenoid : public SolenoidBase {
public:
	explicit Solenoid(UINT32 channel);
	Solenoid(UINT8 moduleNumber, UINT32 channel);
	virtual ~Solenoid();
	virtual void Set(bool on);
	virtual bool Get();

private:
	void InitSolenoid();

	UINT32 m_channel; ///< The channel on the module to control.
};

#endif
