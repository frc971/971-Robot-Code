#include "SensorBase.h"
#include "DigitalModule.h"
#include "Relay.h"
#include "CInterfaces/CRelay.h"

static Relay* relays[SensorBase::kDigitalModules][SensorBase::kRelayChannels];
static bool initialized = false;
static Relay::Direction s_direction = Relay::kBothDirections;

/**
 * Internal function to allocate Relay objects.
 * This function handles the mapping between channel/slot numbers to relay objects. It also
 * allocates Relay objects if they are not already allocated.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The relay channel for this device
 */
static Relay *AllocateRelay(UINT8 moduleNumber, UINT32 channel)
{
	if (!initialized)
	{
		for (unsigned i = 0; i < SensorBase::kDigitalModules; i++)
			for (unsigned j = 0; j < SensorBase::kRelayChannels; j++)
				relays[i][j] = NULL;
		initialized = true;
	}
	if (SensorBase::CheckRelayModule(moduleNumber) && SensorBase::CheckRelayChannel(channel))
	{
		unsigned slotOffset = moduleNumber - 1;
		if (relays[slotOffset][channel - 1] == NULL)
		{
			relays[slotOffset][channel - 1] = new Relay(moduleNumber, channel, s_direction);
		}
		return relays[slotOffset][channel - 1];
	}
	return NULL;
}

/**
 * Set the direction that this relay object will control.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The relay channel number for this object
 * @param direction The direction that the relay object will control
 */
void InitRelay(UINT8 moduleNumber, UINT32 channel, RelayDirection direction)
{
	switch (direction)
	{
	case kBothDirections:
		s_direction = Relay::kBothDirections;
		break;
	case kForwardOnly:
		s_direction = Relay::kForwardOnly;
		break;
	case kReverseOnly:
		s_direction = Relay::kReverseOnly;
		break;
	default:
		s_direction = Relay::kBothDirections;
	}
	AllocateRelay(moduleNumber, channel);
}

/**
 * Set the direction that this relay object will control.
 *
 * @param channel The relay channel number for this object
 * @param direction The direction that the relay object will control
 */
void InitRelay(UINT32 channel, RelayDirection direction)
{
	InitRelay(SensorBase::GetDefaultDigitalModule(), channel, direction);
}

/**
 * Free up the resources associated with this relay.
 * Delete the underlying Relay object and make the channel/port available for reuse.
 *
 * @param slot The slot that the digital module is plugged into
 * @param channel The relay channel number for this object
 */
void DeleteRelay(UINT8 moduleNumber, UINT32 channel)
{
	if (SensorBase::CheckRelayModule(moduleNumber) && SensorBase::CheckRelayChannel(channel))
	{
		unsigned slotOffset = moduleNumber - 1;
		delete relays[slotOffset][channel - 1];
		relays[slotOffset][channel - 1] = NULL;
	}
}

/**
 * Free up the resources associated with this relay.
 * Delete the underlying Relay object and make the channel/port available for reuse.
 *
 * @param channel The relay channel number for this object
 */
void DeleteRelay(UINT32 channel)
{
	DeleteRelay(SensorBase::GetDefaultDigitalModule(), channel);
}

/**
 * Set the relay state.
 *
 * Valid values depend on which directions of the relay are controlled by the object.
 *
 * When set to kBothDirections, the relay can only be one of the three reasonable
 *    values, 0v-0v, 0v-12v, or 12v-0v.
 *
 * When set to kForwardOnly or kReverseOnly, you can specify the constant for the
 *    direction or you can simply specify kOff and kOn.  Using only kOff and kOn is
 *    recommended.
 *
 * @param slot The slot that the digital module is plugged into
 * @param channel The relay channel number for this object
 * @param value The state to set the relay.
 */
void SetRelay(UINT8 moduleNumber, UINT32 channel, RelayValue value)
{
	Relay *relay = AllocateRelay(moduleNumber, channel);
	if (relay != NULL)
	{
		switch (value)
		{
			case kOff: relay->Set(Relay::kOff); break;
			case kOn: relay->Set(Relay::kOn); break;
			case kForward: relay->Set(Relay::kForward); break;
			case kReverse: relay->Set(Relay::kReverse); break;
		}
	}
}

/**
 * Set the relay state.
 *
 * Valid values depend on which directions of the relay are controlled by the object.
 *
 * When set to kBothDirections, the relay can only be one of the three reasonable
 *    values, 0v-0v, 0v-12v, or 12v-0v.
 *
 * When set to kForwardOnly or kReverseOnly, you can specify the constant for the
 *    direction or you can simply specify kOff and kOn.  Using only kOff and kOn is
 *    recommended.
 *
 * @param channel The relay channel number for this object
 * @param value The state to set the relay.
 */
void SetRelay(UINT32 channel, RelayValue value)
{
	SetRelay(SensorBase::GetDefaultDigitalModule(), channel, value);
}


/**
 * Alternate C Interface
 */

RelayObject CreateRelay(UINT8 moduleNumber, UINT32 channel, RelayDirection direction)
{
	switch (direction)
	{
	case kForwardOnly:
		return new Relay(moduleNumber, channel, Relay::kForwardOnly);
	case kReverseOnly:
		return new Relay(moduleNumber, channel, Relay::kReverseOnly);
	case kBothDirections:
	default:
		return new Relay(moduleNumber, channel, Relay::kBothDirections);
	}
}

RelayObject CreateRelay(UINT32 channel, RelayDirection direction)
{
	switch (direction)
	{
	case kForwardOnly:
		return new Relay(channel, Relay::kForwardOnly);
	case kReverseOnly:
		return new Relay(channel, Relay::kReverseOnly);
	case kBothDirections:
	default:
		return new Relay(channel, Relay::kBothDirections);
	}
}

void SetRelay(RelayObject o, RelayValue value)
{
	switch (value)
	{
		case kOff: ((Relay *)o )->Set(Relay::kOff); break;
		case kOn: ((Relay *)o )->Set(Relay::kOn); break;
		case kForward: ((Relay *)o )->Set(Relay::kForward); break;
		case kReverse: ((Relay *)o )->Set(Relay::kReverse); break;
	}
}

void DeleteRelay(RelayObject o)
{
	delete (Relay *) o;
}
