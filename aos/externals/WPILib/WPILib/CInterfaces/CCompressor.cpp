/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "Compressor.h"
#include "CInterfaces/CCompressor.h"
#include "CInterfaces/CError.h"
#include "Utility.h"
#include "WPIErrors.h"
static Compressor *compressor = NULL;

/**
 * Allocate resources for a compressor/pressure switch pair
 * Allocate the underlying object for the compressor.
 * @param pressureSwitchChannel The channel on the default digital module for the pressure switch
 * @param relayChannel The channel on the default digital module for the relay that controls the compressor
 */
void CreateCompressor(UINT32 pressureSwitchChannel, UINT32 relayChannel)
{
	if (compressor == NULL)
	{
		compressor = new Compressor(pressureSwitchChannel, relayChannel);
		return;
	}
	CError *error = new CError();
	wpi_setStaticWPIError(error, CompressorAlreadyDefined);
}

/**
 * Allocate resources for a compressor/pressure switch pair
 * Allocate the underlying object for the compressor.
 * @param pressureSwitchSlot The slot of the digital module for the pressure switch
 * @param pressureSwitchChannel The channel on the digital module for the pressure switch
 * @param relaySlot The slot of the digital module for the relay controlling the compressor
 * @param relayChannel The channel on the digital module for the relay that controls the compressor
 */
void CreateCompressor(UINT32 pressureSwitchSlot, UINT32 pressureSwitchChannel,
						UINT32 relaySlot, UINT32 relayChannel)
{
	if (compressor == NULL)
	{
		compressor = new Compressor(pressureSwitchSlot, pressureSwitchChannel,
									relaySlot, relayChannel);
		return;
	}
	CError *error = new CError();
	wpi_setStaticWPIError(error, CompressorAlreadyDefined);
}

/**
 * Start the compressor
 * Calling this function will cause the compressor task to begin polling the switch and operating the compressor.
 */
void StartCompressor()
{
	if (compressor == NULL)
	{
		CError *error = new CError();
		wpi_setStaticWPIError(error, CompressorUndefined);
		return;
	}
	compressor->Start();
}

/**
 * Stop the compressor.
 * Stops the polling loop that operates the compressor. At this time the compressor will stop operating.
 */
void StopCompressor()
{
	if (compressor == NULL)
	{
		CError *error = new CError();
		wpi_setStaticWPIError(error, CompressorUndefined);
		return;
	}
	compressor->Stop();
}

/**
 * Get the state of the enabled flag.
 * Return the state of the enabled flag for the compressor and pressure switch.
 *
 * @return The state of the compressor task's enable flag.
 */
bool CompressorEnabled()
{
	if (compressor == NULL)
	{
		CError *error = new CError();
		wpi_setStaticWPIError(error, CompressorUndefined);
		return false;
	}
	return compressor->Enabled();
}

/**
 * Free the resources associated with the compressor.
 * The underlying Compressor object will be deleted and the resources and ports freed.
 */
void DeleteCompressor()
{
	delete compressor;
}

