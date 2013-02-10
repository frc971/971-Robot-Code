/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/
#include "CInterfaces/CGearTooth.h"
#include "DigitalModule.h"

static GearTooth* gearToothSensors[SensorBase::kChassisSlots][SensorBase::kDigitalChannels];
static bool initialized = false;

/**
 * Get a pointer to the gear tooth sensor given a slot and a channel.
 * This is an internal routine to allocate (if necessary) a gear tooth
 * object from inputs.
 * @param slot The slot the GearTooth sensor is plugged into.
 * @param channel The channel the GearTooth sensor is plugged into.
 */
static GearTooth *GTptr(UINT8 moduleNumber, UINT32 channel)
{
	if (!initialized)
	{
		initialized = true;
		for (unsigned i = 0; i < SensorBase::kChassisSlots; i++)
			for (unsigned j = 0; j < SensorBase::kDigitalChannels; j++)
				gearToothSensors[i][j] = NULL;
	}
	GearTooth *gt = NULL;
	if (SensorBase::CheckDigitalModule(moduleNumber) && SensorBase::CheckDigitalChannel(channel))
	{
		UINT32 slotIndex = moduleNumber - 1;
		gt = gearToothSensors[slotIndex][channel - 1];
		if (gt == NULL)
		{
			gt = new GearTooth(moduleNumber, channel);
			gearToothSensors[slotIndex][channel - 1] = gt;
		}
	}
	return gt;
}

/**
 * Initialize the gear tooth sensor.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The digital I/O channel the sensor is plugged into
 * @param directionSensitive True if this geartooth sensor can differentiate between
 * foward and backward movement.
 */
void InitGearTooth(UINT8 moduleNumber, UINT32 channel, bool directionSensitive)
{
	GearTooth *gt = GTptr(moduleNumber, channel);
	if (gt) gt->EnableDirectionSensing(directionSensitive);
}

/**
 * Initialize the gear tooth sensor.
 *
 * @param channel The digital I/O channel the sensor is plugged into
 * @param directionSensitive True if this geartooth sensor can differentiate between
 * foward and backward movement.
 */
void InitGearTooth(UINT32 channel, bool directionSensitive)
{
	InitGearTooth(SensorBase::GetDefaultDigitalModule(), channel, directionSensitive);
}

/**
 * Start the GearTooth sensor counting.
 * Start the counting for the geartooth sensor. Before this, the sensor is allocated
 * but not counting pulses.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The digital I/O channel the sensor is plugged into
 */
void StartGearTooth(UINT8 moduleNumber, UINT32 channel)
{
	GearTooth *gt = GTptr(moduleNumber, channel);
	if (gt) gt->Start();
}

/**
 * Start the GearTooth sensor counting.
 * Start the counting for the geartooth sensor. Before this, the sensor is allocated
 * but not counting pulses.
 *
 * @param channel The digital I/O channel the sensor is plugged into
 */
void StartGearTooth(UINT32 channel)
{
	StartGearTooth(SensorBase::GetDefaultDigitalModule(), channel);
}

/**
 * Stop the gear tooth sensor from counting.
 * The counting is disabled on the underlying Counter object.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The digital I/O channel the sensor is plugged into
 */
void StopGearTooth(UINT8 moduleNumber, UINT32 channel)
{
	GearTooth *gt = GTptr(moduleNumber, channel);
	if (gt) gt->Stop();
}

/**
 * Stop the gear tooth sensor from counting.
 * The counting is disabled on the underlying Counter object.
 *
 * @param channel The digital I/O channel the sensor is plugged into
 */
void StopGearTooth(UINT32 channel)
{
	StopGearTooth(SensorBase::GetDefaultDigitalModule(), channel);
}

/**
 * Get value from GearTooth sensor.
 * Get the current count from the sensor.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The digital I/O channel the sensor is plugged into
 */
INT32 GetGearTooth(UINT8 moduleNumber, UINT32 channel)
{
	GearTooth *gt = GTptr(moduleNumber, channel);
	if (gt) return gt->Get();
	return 0;
}

/**
 * Get value from GearTooth sensor.
 * Get the current count from the sensor.
 *
 * @param channel The digital I/O channel the sensor is plugged into
 */
INT32 GetGearTooth(UINT32 channel)
{
	return GetGearTooth(SensorBase::GetDefaultDigitalModule(), channel);
}

/**
 * Reset the GearTooth sensor.
 * Reset the count to zero for the gear tooth sensor.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The digital I/O channel the sensor is plugged into
 */
void ResetGearTooth(UINT8 moduleNumber, UINT32 channel)
{
	GearTooth *gt = GTptr(moduleNumber, channel);
	if (gt) gt->Reset();
}

/**
 * Reset the GearTooth sensor.
 * Reset the count to zero for the gear tooth sensor.
 *
 * @param channel The digital I/O channel the sensor is plugged into
 */
void ResetGearTooth(UINT32 channel)
{
	ResetGearTooth(SensorBase::GetDefaultDigitalModule(), channel);
}

/**
 * Free the resources associated with this gear tooth sensor.
 * Delete the underlying object and free the resources for this geartooth
 * sensor.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The digital I/O channel the sensor is plugged into
 */
void DeleteGearTooth(UINT8 moduleNumber, UINT32 channel)
{
	if (SensorBase::CheckDigitalModule(moduleNumber) && SensorBase::CheckDigitalChannel(channel))
	{
		UINT32 slotIndex = moduleNumber - 1;
		delete gearToothSensors[slotIndex][channel - 1];
		gearToothSensors[slotIndex][channel - 1] = NULL;
	}
}

/**
 * Free the resources associated with this gear tooth sensor.
 * Delete the underlying object and free the resources for this geartooth
 * sensor.
 *
 * @param channel The digital I/O channel the sensor is plugged into
 */
void DeleteGearTooth(UINT32 channel)
{
	DeleteGearTooth(SensorBase::GetDefaultDigitalModule(), channel);
}


/**
 * Alternate C Interface
 */

GearToothObject CreateGearTooth(UINT32 channel, bool directionSensitive)
{
	return (GearToothObject) new GearTooth(channel, directionSensitive);
}

GearToothObject CreateGearTooth(UINT8 moduleNumber, UINT32 channel, bool directionSensitive)
{
	return (GearToothObject) new GearTooth(moduleNumber, channel, directionSensitive);
}

void StartGearTooth(GearToothObject o)
{
	((GearTooth *)o )->Start();
}

void StopGearTooth(GearToothObject o)
{
	((GearTooth *)o )->Stop();
}

INT32 GetGearTooth(GearToothObject o)
{
	return ((GearTooth *)o )->Get();
}

void ResetGearTooth(GearToothObject o)
{
	((GearTooth *)o )->Reset();
}

void DeleteGearTooth(GearToothObject o)
{
	delete (GearTooth *)o;
}

