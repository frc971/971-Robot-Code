/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "SensorBase.h"

#include "NetworkCommunication/LoadOut.h"
#include "WPIErrors.h"

const UINT32 SensorBase::kSystemClockTicksPerMicrosecond;
const UINT32 SensorBase::kDigitalChannels;
const UINT32 SensorBase::kAnalogChannels;
const UINT32 SensorBase::kAnalogModules;
const UINT32 SensorBase::kDigitalModules;
const UINT32 SensorBase::kSolenoidChannels;
const UINT32 SensorBase::kSolenoidModules;
const UINT32 SensorBase::kPwmChannels;
const UINT32 SensorBase::kRelayChannels;
const UINT32 SensorBase::kChassisSlots;
SensorBase *SensorBase::m_singletonList = NULL;

/**
 * Creates an instance of SensorBase.
 */
SensorBase::SensorBase()
{
}

/**
 * Frees the resources for a SensorBase.
 */
SensorBase::~SensorBase()
{
}

/**
 * @brief Add sensor to the singleton list.
 * Add this object to the list of singletons that need to be deleted when
 * the robot program exits. Each of the objects on this list are singletons,
 * that is they aren't allocated directly by user code, but instead are
 * allocated by (for example) a static GetInstance method. Because of this, they
 * need some way to be freed when the module is unloaded so that they can free
 * any resources that they are holding on to.
 * @see #DeleteSingletons()
 */
void SensorBase::AddToSingletonList()
{
	m_nextSingleton = m_singletonList;
	m_singletonList = this;
}

/**
 * @brief Delete all the singleton objects on the list.
 * All the objects that were allocated as singletons need to be deleted so
 * their resources can be freed.
 * @see #AddToSingletonList()
 */
void SensorBase::DeleteSingletons()
{
	for (SensorBase *next = m_singletonList; next != NULL;)
	{
		SensorBase *tmp = next;
		next = next->m_nextSingleton;
		delete tmp;
	}
	m_singletonList = NULL;
}

/**
 * Check that the analog module number is valid.
 * 
 * @return Analog module is valid and present
 */
bool SensorBase::CheckAnalogModule(UINT8 moduleNumber)
{
	if (nLoadOut::getModulePresence(nLoadOut::kModuleType_Analog, moduleNumber - 1))
		return true;
	return false;
}

/**
 * Check that the digital module number is valid.
 * 
 * @return Digital module is valid and present
 */
bool SensorBase::CheckDigitalModule(UINT8 moduleNumber)
{
	if (nLoadOut::getModulePresence(nLoadOut::kModuleType_Digital, moduleNumber - 1))
		return true;
	return false;
}

/**
 * Check that the digital module number is valid.
 * 
 * @return Digital module is valid and present
 */
bool SensorBase::CheckPWMModule(UINT8 moduleNumber)
{
	return CheckDigitalModule(moduleNumber);
}

/**
 * Check that the digital module number is valid.
 * 
 * @return Digital module is valid and present
 */
bool SensorBase::CheckRelayModule(UINT8 moduleNumber)
{
	return CheckDigitalModule(moduleNumber);
}

/**
 * Check that the solenoid module number is valid.
 * 
 * @return Solenoid module is valid and present
 */
bool SensorBase::CheckSolenoidModule(UINT8 moduleNumber)
{
	if (nLoadOut::getModulePresence(nLoadOut::kModuleType_Solenoid, moduleNumber - 1))
		return true;
	return false;
}

/**
 * Check that the digital channel number is valid.
 * Verify that the channel number is one of the legal channel numbers. Channel numbers are
 * 1-based.
 * 
 * @return Digital channel is valid
 */
bool SensorBase::CheckDigitalChannel(UINT32 channel)
{
	if (channel > 0 && channel <= kDigitalChannels)
		return true;
	return false;
}

/**
 * Check that the digital channel number is valid.
 * Verify that the channel number is one of the legal channel numbers. Channel numbers are
 * 1-based.
 * 
 * @return Relay channel is valid
 */
bool SensorBase::CheckRelayChannel(UINT32 channel)
{
	if (channel > 0 && channel <= kRelayChannels)
		return true;
	return false;
}

/**
 * Check that the digital channel number is valid.
 * Verify that the channel number is one of the legal channel numbers. Channel numbers are
 * 1-based.
 * 
 * @return PWM channel is valid
 */
bool SensorBase::CheckPWMChannel(UINT32 channel)
{
	if (channel > 0 && channel <= kPwmChannels)
		return true;
	return false;
}

/**
 * Check that the analog channel number is value.
 * Verify that the analog channel number is one of the legal channel numbers. Channel numbers
 * are 1-based.
 * 
 * @return Analog channel is valid
 */
bool SensorBase::CheckAnalogChannel(UINT32 channel)
{
	if (channel > 0 && channel <= kAnalogChannels)
		return true;
	return false;
}

/**
 * Verify that the solenoid channel number is within limits.
 * 
 * @return Solenoid channel is valid
 */
bool SensorBase::CheckSolenoidChannel(UINT32 channel)
{
	if (channel > 0 && channel <= kSolenoidChannels)
		return true;
	return false;
}

