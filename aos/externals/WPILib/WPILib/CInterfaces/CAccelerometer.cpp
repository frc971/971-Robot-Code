/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "Accelerometer.h"
#include "CInterfaces/CAccelerometer.h"
#include "AnalogModule.h"
#include "CInterfaces/CWrappers.h"

static Accelerometer* accelerometers[SensorBase::kAnalogModules][SensorBase::kAnalogChannels];
static bool initialized = false;

/**
 * Allocate an instance of the C Accelerometer object
 * @param slot The slot the analog module is plugged into
 * @param channel The analog module channel the accelerometer is plugged into
 */
static Accelerometer *AllocateAccelerometer(UINT8 moduleNumber, UINT32 channel)
{
	if (!initialized)
	{
		initialized = true;
		for (unsigned i = 1; i <= SensorBase::kAnalogModules; i++)
			for (unsigned j = 1; j <= SensorBase::kAnalogChannels; j++)
				accelerometers[i][j] = NULL;
	}
	if (!SensorBase::CheckAnalogModule(moduleNumber) || !SensorBase::CheckAnalogChannel(channel))
		return NULL;
	unsigned index = moduleNumber - 1;
	if (accelerometers[index][channel - 1] == NULL)
	{
		accelerometers[index][channel - 1] = new Accelerometer(moduleNumber, channel);
	}
	return accelerometers[index][channel - 1];
}

/**
 * Get the acceleration in Gs
 * @param channel The channel the accelerometer is plugged into
 * @returns Returns the acceleration in Gs
 */
float GetAcceleration(UINT32 channel)
{
	Accelerometer *accel = AllocateAccelerometer(SensorBase::GetDefaultAnalogModule(), channel);
	return accel->GetAcceleration();
}

/**
 * Get the acceleration in Gs
 * @param slot The slot the analog module is plugged into
 * @param channel The channel the accelerometer is plugged into
 * @returns Returns the acceleration in Gs
 */
float GetAcceleration(UINT8 moduleNumber, UINT32 channel)
{
	Accelerometer *accel = AllocateAccelerometer(moduleNumber, channel);
	return accel->GetAcceleration();
}

/**
 * Set the accelerometer sensitivity.
 *
 * This sets the sensitivity of the accelerometer used for calculating the acceleration.
 * The sensitivity varys by accelerometer model. There are constants defined for various models.
 *
 * @param channel The channel the accelerometer is plugged into
 * @param sensitivity The sensitivity of accelerometer in Volts per G.
 */
void SetAccelerometerSensitivity(UINT32 channel, float sensitivity)
{
	Accelerometer *accel = AllocateAccelerometer(SensorBase::GetDefaultAnalogModule(), channel);
	accel->SetSensitivity(sensitivity);
}

/**
 * Set the accelerometer sensitivity.
 *
 * This sets the sensitivity of the accelerometer used for calculating the acceleration.
 * The sensitivity varys by accelerometer model. There are constants defined for various models.
 *
 * @param slot The slot the analog module is plugged into
 * @param channel The channel the accelerometer is plugged into
 * @param sensitivity The sensitivity of accelerometer in Volts per G.
 */
void SetAccelerometerSensitivity(UINT8 moduleNumber, UINT32 channel, float sensitivity)
{
	Accelerometer *accel = AllocateAccelerometer(moduleNumber, channel);
	accel->SetSensitivity(sensitivity);
}

/**
 * Set the voltage that corresponds to 0 G.
 *
 * The zero G voltage varys by accelerometer model. There are constants defined for various models.
 *
 * @param channel The channel the accelerometer is plugged into
 * @param zero The zero G voltage.
 */
void SetAccelerometerZero(UINT32 channel, float zero)
{
	Accelerometer *accel = AllocateAccelerometer(SensorBase::GetDefaultAnalogModule(), channel);
	accel->SetZero(zero);
}

/**
 * Set the voltage that corresponds to 0 G.
 *
 * The zero G voltage varys by accelerometer model. There are constants defined for various models.
 *
 * @param slot The slot the analog module is plugged into
 * @param channel The channel the accelerometer is plugged into
 * @param zero The zero G voltage.
 */
void SetAccelerometerZero(UINT8 moduleNumber, UINT32 channel, float zero)
{
	Accelerometer *accel = AllocateAccelerometer(moduleNumber, channel);
	accel->SetZero(zero);
}

/**
 * Delete the accelerometer underlying object
 * Deletes the object that is associated with this accelerometer and frees up the storage
 * and the ports.
 *
 * @param slot The slot the analog module is plugged into
 * @param channel The channel the accelerometer is plugged into
 */
void DeleteAccelerometer(UINT8 moduleNumber, UINT32 channel)
{
	if (SensorBase::CheckAnalogModule(moduleNumber) && SensorBase::CheckAnalogChannel(channel))
	{
		unsigned index = moduleNumber - 1;
		delete accelerometers[index][channel - 1];
		accelerometers[index][channel - 1] = NULL;
	}
}

/**
 * Delete the accelerometer underlying object
 * Deletes the object that is associated with this accelerometer and frees up the storage
 * and the ports.
 *
 * @param channel The channel the accelerometer is plugged into
 */
void DeleteAccelerometer(UINT32 channel)
{
	DeleteAccelerometer(SensorBase::GetDefaultAnalogModule(), channel);
}


/**
 * Alternate C Interface
 */

AccelerometerObject CreateAccelerometer(UINT32 channel)
{
	return (AccelerometerObject) new Accelerometer(channel);
}

AccelerometerObject CreateAccelerometer(UINT8 moduleNumber, UINT32 channel)
{
	return (AccelerometerObject) new Accelerometer(moduleNumber, channel);
}

float GetAcceleration(AccelerometerObject o)
{
	return ((Accelerometer *) o)->GetAcceleration();
}

void SetAccelerometerSensitivity(AccelerometerObject o, float sensitivity)
{
	((Accelerometer *) o)->SetSensitivity(sensitivity);
}

void SetAccelerometerZero(AccelerometerObject o, float zero)
{
	((Accelerometer *)o )->SetZero(zero);
}

void DeleteAccelerometer(AccelerometerObject o)
{
	delete (Accelerometer *) o;
}

