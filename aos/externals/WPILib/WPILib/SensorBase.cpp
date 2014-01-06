/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "SensorBase.h"

#include "NetworkCommunication/LoadOut.h"

const uint32_t SensorBase::kSystemClockTicksPerMicrosecond;
const uint32_t SensorBase::kDigitalChannels;
const uint32_t SensorBase::kAnalogChannels;
const uint32_t SensorBase::kAnalogModules;
const uint32_t SensorBase::kDigitalModules;
const uint32_t SensorBase::kSolenoidChannels;
const uint32_t SensorBase::kSolenoidModules;
const uint32_t SensorBase::kPwmChannels;
const uint32_t SensorBase::kRelayChannels;
SensorBase *SensorBase::m_singletonList = NULL;
ReentrantSemaphore SensorBase::m_singletonListSemaphore;

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
  Synchronized sync(m_singletonListSemaphore);
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
  Synchronized sync(m_singletonListSemaphore);
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
bool SensorBase::CheckAnalogModule(uint8_t moduleNumber)
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
bool SensorBase::CheckDigitalModule(uint8_t moduleNumber)
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
bool SensorBase::CheckPWMModule(uint8_t moduleNumber)
{
	return CheckDigitalModule(moduleNumber);
}

/**
 * Check that the digital module number is valid.
 * 
 * @return Digital module is valid and present
 */
bool SensorBase::CheckRelayModule(uint8_t moduleNumber)
{
	return CheckDigitalModule(moduleNumber);
}

/**
 * Check that the solenoid module number is valid.
 * 
 * @return Solenoid module is valid and present
 */
bool SensorBase::CheckSolenoidModule(uint8_t moduleNumber)
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
bool SensorBase::CheckDigitalChannel(uint32_t channel)
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
bool SensorBase::CheckRelayChannel(uint32_t channel)
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
bool SensorBase::CheckPWMChannel(uint32_t channel)
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
bool SensorBase::CheckAnalogChannel(uint32_t channel)
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
bool SensorBase::CheckSolenoidChannel(uint32_t channel)
{
	if (channel > 0 && channel <= kSolenoidChannels)
		return true;
	return false;
}

