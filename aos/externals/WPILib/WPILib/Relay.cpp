/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "Relay.h"

#include "DigitalModule.h"
#include "NetworkCommunication/UsageReporting.h"
#include "Resource.h"
#include "WPIErrors.h"

// Allocate each direction separately.
static Resource *relayChannels = NULL;

/**
 * Common relay intitialization methode.
 * This code is common to all Relay constructors and initializes the relay and reserves
 * all resources that need to be locked. Initially the relay is set to both lines at 0v.
 * @param slot The module slot number this relay is connected to.
 * 
 * @param moduleNumber The digital module this relay is connected to (1 or 2).
 */
void Relay::InitRelay (UINT8 moduleNumber)
{
	char buf[64];
	Resource::CreateResourceObject(&relayChannels, tDIO::kNumSystems * kRelayChannels * 2);
	if (!SensorBase::CheckRelayModule(moduleNumber))
	{
		snprintf(buf, 64, "Digital Module %d", moduleNumber);
		wpi_setWPIErrorWithContext(ModuleIndexOutOfRange, buf);
		return;
	}
	if (!SensorBase::CheckRelayChannel(m_channel))
	{
		snprintf(buf, 64, "Relay Channel %d", m_channel);
		wpi_setWPIErrorWithContext(ChannelIndexOutOfRange, buf);
		return;
	}

	if (m_direction == kBothDirections || m_direction == kForwardOnly)
	{
		snprintf(buf, 64, "Forward Relay %d (Module: %d)", m_channel, moduleNumber);
		if (relayChannels->Allocate(((moduleNumber - 1) * kRelayChannels + m_channel - 1) * 2, buf) == ~0ul)
		{
			CloneError(relayChannels);
			return;
		}

		nUsageReporting::report(nUsageReporting::kResourceType_Relay, m_channel, moduleNumber - 1);
	}
	if (m_direction == kBothDirections || m_direction == kReverseOnly)
	{
		snprintf(buf, 64, "Reverse Relay %d (Module: %d)", m_channel, moduleNumber);
		if (relayChannels->Allocate(((moduleNumber - 1) * kRelayChannels + m_channel - 1) * 2 + 1, buf) == ~0ul)
		{
			CloneError(relayChannels);
			return;
		}

		nUsageReporting::report(nUsageReporting::kResourceType_Relay, m_channel + 128, moduleNumber - 1);
	}
	m_module = DigitalModule::GetInstance(moduleNumber);
	m_module->SetRelayForward(m_channel, false);
	m_module->SetRelayReverse(m_channel, false);
}

/**
 * Relay constructor given the module and the channel.
 * 
 * @param moduleNumber The digital module this relay is connected to (1 or 2).
 * @param channel The channel number within the module for this relay.
 * @param direction The direction that the Relay object will control.
 */
Relay::Relay(UINT8 moduleNumber, UINT32 channel, Relay::Direction direction)
	: m_channel (channel)
	, m_direction (direction)
{
	InitRelay(moduleNumber);
}

/**
 * Relay constructor given a channel only where the default digital module is used.
 * @param channel The channel number within the default module for this relay.
 * @param direction The direction that the Relay object will control.
 */
Relay::Relay(UINT32 channel, Relay::Direction direction)
	: m_channel (channel)
	, m_direction (direction)
{
	InitRelay(GetDefaultDigitalModule());
}

/**
 * Free the resource associated with a relay.
 * The relay channels are set to free and the relay output is turned off.
 */
Relay::~Relay()
{
	m_module->SetRelayForward(m_channel, false);
	m_module->SetRelayReverse(m_channel, false);

	if (m_direction == kBothDirections || m_direction == kForwardOnly)
	{
		relayChannels->Free(((m_module->GetNumber() - 1) * kRelayChannels + m_channel - 1) * 2);
	}
	if (m_direction == kBothDirections || m_direction == kReverseOnly)
	{
		relayChannels->Free(((m_module->GetNumber() - 1) * kRelayChannels + m_channel - 1) * 2 + 1);
	}
}

/**
 * Set the relay state.
 * 
 * Valid values depend on which directions of the relay are controlled by the object.
 * 
 * When set to kBothDirections, the relay can be any of the four states:
 *    0v-0v, 0v-12v, 12v-0v, 12v-12v
 * 
 * When set to kForwardOnly or kReverseOnly, you can specify the constant for the
 *    direction or you can simply specify kOff and kOn.  Using only kOff and kOn is
 *    recommended.
 * 
 * @param value The state to set the relay.
 */
void Relay::Set(Relay::Value value)
{
	if (StatusIsFatal()) return;
	switch (value)
	{
	case kOff:
		if (m_direction == kBothDirections || m_direction == kForwardOnly)
		{
			m_module->SetRelayForward(m_channel, false);
		}
		if (m_direction == kBothDirections || m_direction == kReverseOnly)
		{
			m_module->SetRelayReverse(m_channel, false);
		}
		break;
	case kOn:
		if (m_direction == kBothDirections || m_direction == kForwardOnly)
		{
			m_module->SetRelayForward(m_channel, true);
		}
		if (m_direction == kBothDirections || m_direction == kReverseOnly)
		{
			m_module->SetRelayReverse(m_channel, true);
		}
		break;
	case kForward:
		if (m_direction == kReverseOnly)
		{
			wpi_setWPIError(IncompatibleMode);
			break;
		}
		if (m_direction == kBothDirections || m_direction == kForwardOnly)
		{
			m_module->SetRelayForward(m_channel, true);
		}
		if (m_direction == kBothDirections)
		{
			m_module->SetRelayReverse(m_channel, false);
		}
		break;
	case kReverse:
		if (m_direction == kForwardOnly)
		{
			wpi_setWPIError(IncompatibleMode);
			break;
		}
		if (m_direction == kBothDirections)
		{
			m_module->SetRelayForward(m_channel, false);
		}
		if (m_direction == kBothDirections || m_direction == kReverseOnly)
		{
			m_module->SetRelayReverse(m_channel, true);
		}
		break;
	}
}
