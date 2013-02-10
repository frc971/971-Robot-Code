/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "Solenoid.h"
#include "CInterfaces/CSolenoid.h"

static Solenoid *solenoids[SensorBase::kSolenoidChannels];
static bool initialized = false;

/**
 * Internal allocation function for Solenoid channels.
 * The function is used interally to allocate the Solenoid object and keep track
 * of the channel mapping to the object for subsequent calls.
 *
 * @param channel The channel for the solenoid
 */
static Solenoid *allocateSolenoid(UINT32 channel)
{
	if (!initialized)
	{
		initialized = true;
		for (unsigned i = 0; i < SensorBase::kSolenoidChannels; i++)
			solenoids[i] = new Solenoid(i + 1);
	}
	if (channel < 1 || channel > SensorBase::kSolenoidChannels)
		return NULL;
	return solenoids[channel - 1];
}

/**
 * Set the value of a solenoid.
 *
 * @param channel The channel on the Solenoid module
 * @param on Turn the solenoid output off or on.
 */
void SetSolenoid(UINT32 channel, bool on)
{
	Solenoid *s = allocateSolenoid(channel);
	if (s != NULL)
		s->Set(on);
}

/**
 * Read the current value of the solenoid.
 *
 * @param channel The channel in the Solenoid module
 * @return The current value of the solenoid.
 */
bool GetSolenoid(UINT32 channel)
{
	Solenoid *s = allocateSolenoid(channel);
	if (s == NULL)
		return false;
	return s->Get();
}

/**
 * Free the resources associated with the Solenoid channel.
 * Free the resources including the Solenoid object for this channel.
 *
 * @param channel The channel in the Solenoid module
 */
void DeleteSolenoid(UINT32 channel)
{
	if (channel >= 1 && channel <= SensorBase::kSolenoidChannels)
	{
		delete solenoids[channel - 1];
		solenoids[channel - 1] = NULL;
	}
}

SolenoidObject CreateSolenoid(UINT32 channel)
{
	return (SolenoidObject) new Solenoid(channel);
}

void DeleteSolenoid(SolenoidObject o)
{
	delete (Solenoid *) o;
}

void SetSolenoid(SolenoidObject o, bool on)
{
	((Solenoid *)o)->Set(on);
}

bool GetSolenoid(SolenoidObject o)
{
	return ((Solenoid *)o)->Get();
}
