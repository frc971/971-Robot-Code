/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "CInterfaces/CGyro.h"
#include "Gyro.h"

static Gyro* gyros[2] = {NULL, NULL};

/**
 * Allocate resoures for a Gyro.
 *
 * This is an internal routine and not used outside of this module.
 *
 * @param slot The analog module that the gyro is connected to. Must be slot 1 on the current
 * hardware implementation.
 * @param channel The analog channel the gyro is connected to. Must be channel 1 or 2 only (the only
 * ones with the attached accumulator)
 */
static Gyro *AllocateGyro(UINT32 slot, UINT32 channel)
{
	Gyro *gyro = NULL;
	if (slot == 1 && (channel == 1 || channel == 2))
	{
		if ((gyro = gyros[channel - 1]) == NULL)
		{
			gyro = new Gyro(channel);
			gyros[channel - 1] = gyro;
		}
	}
	return gyro;
}

/**
 * Initialize the gyro.
 * Calibrate the gyro by running for a number of samples and computing the center value for this
 * part. Then use the center value as the Accumulator center value for subsequent measurements.
 * It's important to make sure that the robot is not moving while the centering calculations are
 * in progress, this is typically done when the robot is first turned on while it's sitting at
 * rest before the competition starts.
 *
 * @param slot The slot the analog module is connected to
 * @param channel The analog channel the gyro is plugged into
 */
void InitGyro(UINT32 slot, UINT32 channel)
{
	AllocateGyro(slot, channel);
}

/**
 * Initialize the gyro.
 * Calibrate the gyro by running for a number of samples and computing the center value for this
 * part. Then use the center value as the Accumulator center value for subsequent measurements.
 * It's important to make sure that the robot is not moving while the centering calculations are
 * in progress, this is typically done when the robot is first turned on while it's sitting at
 * rest before the competition starts.
 *
 * @param channel The analog channel the gyro is plugged into
 */
void InitGyro(UINT32 channel)
{
	InitGyro(SensorBase::GetDefaultAnalogModule(), channel);
}

/**
 * Return the actual angle in degrees that the robot is currently facing.
 *
 * The angle is based on the current accumulator value corrected by the oversampling rate, the
 * gyro type and the A/D calibration values.
 * The angle is continuous, that is can go beyond 360 degrees. This make algorithms that wouldn't
 * want to see a discontinuity in the gyro output as it sweeps past 0 on the second time around.
 *
 * @param slot The slot the analog module is connected to
 * @param channel The analog channel the gyro is plugged into
 * @return the current heading of the robot in degrees. This heading is based on integration
 * of the returned rate from the gyro.
 */
float GetGyroAngle(UINT32 slot, UINT32 channel)
{
	Gyro *gyro = AllocateGyro(slot, channel);
	if (gyro) return gyro->GetAngle();
	return 0.0;
}

/**
 * Return the actual angle in degrees that the robot is currently facing.
 *
 * The angle is based on the current accumulator value corrected by the oversampling rate, the
 * gyro type and the A/D calibration values.
 * The angle is continuous, that is can go beyond 360 degrees. This make algorithms that wouldn't
 * want to see a discontinuity in the gyro output as it sweeps past 0 on the second time around.
 *
 * @param channel The analog channel the gyro is plugged into
 * @return the current heading of the robot in degrees. This heading is based on integration
 * of the returned rate from the gyro.
 */
float GetGyroAngle(UINT32 channel)
{
	return GetGyroAngle(SensorBase::GetDefaultAnalogModule(), channel);
}

/**
 * Reset the gyro.
 * Resets the gyro to a heading of zero. This can be used if there is significant
 * drift in the gyro and it needs to be recalibrated after it has been running.

 * @param slot The slot the analog module is connected to
 * @param channel The analog channel the gyro is plugged into
 */
void ResetGyro(UINT32 slot, UINT32 channel)
{
	Gyro *gyro = AllocateGyro(slot, channel);
	if (gyro) gyro->Reset();
}

/**
 * Reset the gyro.
 * Resets the gyro to a heading of zero. This can be used if there is significant
 * drift in the gyro and it needs to be recalibrated after it has been running.

 * @param channel The analog channel the gyro is plugged into
 */
void ResetGyro(UINT32 channel)
{
	ResetGyro(SensorBase::GetDefaultAnalogModule(), channel);
}

/**
 * Set the gyro type based on the sensitivity.
 * This takes the number of volts/degree/second sensitivity of the gyro and uses it in subsequent
 * calculations to allow the code to work with multiple gyros.
 *
 * @param slot The slot the analog module is connected to
 * @param channel The analog channel the gyro is plugged into
 * @param voltsPerDegreePerSecond The type of gyro specified as the voltage that represents one degree/second.
 */
void SetGyroSensitivity(UINT32 slot, UINT32 channel, float voltsPerDegreePerSecond)
{
	Gyro *gyro = AllocateGyro(slot, channel);
	if (gyro) gyro->SetSensitivity(voltsPerDegreePerSecond);
}

/**
 * Set the gyro type based on the sensitivity.
 * This takes the number of volts/degree/second sensitivity of the gyro and uses it in subsequent
 * calculations to allow the code to work with multiple gyros.
 *
 * @param channel The analog channel the gyro is plugged into
 * @param voltsPerDegreePerSecond The type of gyro specified as the voltage that represents one degree/second.
 */
void SetGyroSensitivity(UINT32 channel, float voltsPerDegreePerSecond)
{
	SetGyroSensitivity(SensorBase::GetDefaultAnalogModule(), channel, voltsPerDegreePerSecond);
}

/**
 * Free the resources associated with this Gyro
 * Free the Gyro object and the reservation for this slot/channel.
 *
 * @param slot The slot the analog module is connected to
 * @param channel The analog channel the gyro is plugged into
 */
void DeleteGyro(UINT32 slot, UINT32 channel)
{
	if (slot == 1 && (channel == 1 || channel == 2))
	{
		delete gyros[channel - 1];
		gyros[channel - 1] = NULL;
	}
}

void DeleteGyro(UINT32 channel)
{
	DeleteGyro(SensorBase::GetDefaultAnalogModule(), channel);
}

/**
 * Alternate C interface to Gyro
 */

GyroObject CreateGyro(UINT32 slot, UINT32 channel)
{
	return (GyroObject) new Gyro(slot, channel);
}

GyroObject CreateGyro(UINT32 channel)
{
	return (GyroObject) new Gyro(channel);
}

float GetGyroAngle(GyroObject o)
{
	return ((Gyro *) o)->GetAngle();
}

void ResetGyro(GyroObject o)
{
	((Gyro *) o)->Reset();
}

void SetGyroSensitivity(GyroObject o, float voltsPerDegreePerSecond)
{
	((Gyro *) o)->SetSensitivity(voltsPerDegreePerSecond);
}

void Delete(GyroObject o)
{
	delete ((Gyro *) o);
}

