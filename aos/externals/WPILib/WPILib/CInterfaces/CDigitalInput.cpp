/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "DigitalModule.h"
#include "DigitalInput.h"
#include "CInterfaces/CDigitalInput.h"

static DigitalInput* digitalInputs[SensorBase::kDigitalModules][SensorBase::kDigitalChannels];
static bool initialized = false;

/**
 * Allocates the resources associated with a DigitalInput.
 * Allocate the underlying DigitalInput object and the reservations for the
 * associated slot and channel.
 *
 * @param slot The slot the digital input module is plugged into
 * @param channel The particular channel this digital input is using
 */
static DigitalInput *AllocateDigitalInput(UINT8 moduleNumber, UINT32 channel)
{
	if (!initialized)
	{
		for (unsigned i = 0; i < SensorBase::kDigitalModules; i++)
			for (unsigned j = 0; j < SensorBase::kDigitalChannels; j++)
				digitalInputs[i][j] = NULL;
		initialized = true;
	}
	if (SensorBase::CheckDigitalModule(moduleNumber) && SensorBase::CheckDigitalChannel(channel))
	{
		UINT32 slotOffset = moduleNumber - 1;
		if (digitalInputs[slotOffset][channel - 1] == NULL)
		{
			digitalInputs[slotOffset][channel - 1] = new DigitalInput(moduleNumber, channel);
		}
		return digitalInputs[slotOffset][channel - 1];
	}
	return NULL;
}

/*
 * Get the value from a digital input channel.
 * Retrieve the value of a single digital input channel from the FPGA.
 *
 * @param slot The slot the digital input module is plugged into
 * @param channel The particular channel this digital input is using
 */
UINT32 GetDigitalInput(UINT8 moduleNumber, UINT32 channel)
{
	DigitalInput *digIn = AllocateDigitalInput(moduleNumber, channel);
	if (digIn)
		return digIn->Get();
	else
		return 0;
}

/*
 * Get the value from a digital input channel.
 * Retrieve the value of a single digital input channel from the FPGA.
 *
 * @param channel The particular channel this digital input is using
 */
UINT32 GetDigitalInput(UINT32 channel)
{
	return GetDigitalInput(SensorBase::GetDefaultDigitalModule(), channel);
}

/**
 * Frees the resources for this DigitalInput.
 * Deletes the underlying object and frees the reservation for the associated digital
 * input port.
 *
 * @param slot The slot the digital input module is plugged into
 * @param channel The particular channel this digital input is using
 */
void DeleteDigitalInput(UINT8 moduleNumber, UINT32 channel)
{
	if (SensorBase::CheckDigitalModule(moduleNumber) && SensorBase::CheckDigitalChannel(channel))
	{
		UINT32 slotOffset = moduleNumber - 1;
		delete digitalInputs[slotOffset][channel - 1];
		digitalInputs[slotOffset][channel - 1] = NULL;
	}
}

/**
 * Frees the resources for this DigitalInput.
 * Deletes the underlying object and frees the reservation for the associated digital
 * input port.
 *
 * @param channel The particular channel this digital input is using
 */
void DeleteDigitalInput(UINT32 channel)
{
	DeleteDigitalInput(SensorBase::GetDefaultDigitalModule(), channel);
}

/*******************************************************************************
 * Alternative interface to digital input
*******************************************************************************/
DigitalInputObject CreateDigitalInput(UINT8 moduleNumber, UINT32 channel)
{
	return (DigitalInputObject) new DigitalInput(moduleNumber, channel); 
}

DigitalInputObject CreateDigitalInput(UINT32 channel)
{
	return (DigitalInputObject) new DigitalInput(channel); 
}

bool GetDigitalInput(DigitalInputObject o)
{
	return ((DigitalInput *) o)->Get();
}

void DeleteDigitalInput(DigitalInputObject o)
{
	delete (DigitalInput *) o;
}
