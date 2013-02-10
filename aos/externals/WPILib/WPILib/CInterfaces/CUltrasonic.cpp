/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/
#include "CInterfaces/CUltrasonic.h"
#include "DigitalModule.h"

static Ultrasonic* ultrasonics[SensorBase::kChassisSlots][SensorBase::kDigitalChannels];
static bool initialized = false;

/**
 * Internal routine to allocate and initialize resources for an Ultrasonic sensor
 * Allocate the actual Ultrasonic sensor object and the slot/channels associated with them. Then
 * initialize the sensor.
 *
 * @param pingSlot The slot for the digital module for the ping connection
 * @param pingChannel The channel on the digital module for the ping connection
 * @param echoSlot The slot for the digital module for the echo connection
 * @param echoChannel The channel on the digital module for the echo connection
 */
static void USinit(UINT8 pingModuleNumber, UINT32 pingChannel, UINT8 echoModuleNumber, UINT32 echoChannel)
{
	if (!initialized)
	{
		initialized = true;
		for (unsigned i = 0; i < SensorBase::kChassisSlots; i++)
			for (unsigned j = 0; j < SensorBase::kDigitalChannels; j++)
				ultrasonics[i][j] = NULL;
	}
	if (SensorBase::CheckDigitalModule(pingModuleNumber)
			&& SensorBase::CheckDigitalChannel(pingChannel)
		 	&& SensorBase::CheckDigitalModule(echoModuleNumber)
			&& SensorBase::CheckDigitalChannel(echoChannel))
	{
		if (ultrasonics[pingModuleNumber - 1][pingChannel - 1] == NULL)
		{
			ultrasonics[pingModuleNumber - 1][pingChannel - 1] = new Ultrasonic(pingModuleNumber, pingChannel, echoModuleNumber, echoChannel);
			printf("new Ultrasonic(%d, %d, %d, %d)\n", pingModuleNumber, pingChannel, echoModuleNumber, echoChannel);
		}
		Ultrasonic::SetAutomaticMode(true);
	}
}

/**
 * Internal routine to return the pointer to an Ultrasonic sensor
 * Return the pointer to a previously allocated Ultrasonic sensor object. Only the ping connection
 * is required since there can only be a single sensor connected to that channel
 *
 * @param pingSlot The slot for the digital module for the ping connection
 * @param pingChannel The channel on the digital module for the ping connection
 */
static Ultrasonic *USptr(UINT8 pingModuleNumber, UINT32 pingChannel)
{
	if (!SensorBase::CheckDigitalModule(pingModuleNumber) || !SensorBase::CheckDigitalChannel(pingChannel))
		return NULL;
	Ultrasonic *us = ultrasonics[pingModuleNumber - 1][pingChannel - 1];
	return us;
}

/**
 * Initialize and Ultrasonic sensor.
 *
 * Initialize an Ultrasonic sensor to start it pinging in round robin mode with other allocated
 * sensors. There is no need to explicitly start the sensor pinging.
 *
 * @param pingSlot The slot for the digital module for the ping connection
 * @param pingChannel The channel on the digital module for the ping connection
 * @param echoSlot The slot for the digital module for the echo connection
 * @param echoChannel The channel on the digital module for the echo connection
 */
void InitUltrasonic(UINT8 pingModuleNumber, UINT32 pingChannel, UINT8 echoModuleNumber, UINT32 echoChannel)
{
	Ultrasonic *us = USptr(pingModuleNumber, pingChannel);
	if (!us) USinit(pingModuleNumber, pingChannel, echoModuleNumber, echoChannel);
}

/**
 * Initialize and Ultrasonic sensor.
 *
 * Initialize an Ultrasonic sensor to start it pinging in round robin mode with other allocated
 * sensors. There is no need to explicitly start the sensor pinging.
 *
 * @param pingChannel The channel on the digital module for the ping connection
 * @param echoChannel The channel on the digital module for the echo connection
 */
void InitUltrasonic(UINT32 pingChannel, UINT32 echoChannel)
{
	InitUltrasonic(SensorBase::GetDefaultDigitalModule(), pingChannel,
				   SensorBase::GetDefaultDigitalModule(), echoChannel);
}

/**
 * Get the range in inches from the ultrasonic sensor.
 * @return double Range in inches of the target returned from the ultrasonic sensor. If there is
 * no valid value yet, i.e. at least one measurement hasn't completed, then return 0.
 *
 * @param pingSlot The slot for the digital module for the ping connection
 * @param pingChannel The channel on the digital module for the ping connection
 * @param echoSlot The slot for the digital module for the echo connection
 * @param echoChannel The channel on the digital module for the echo connection
 */
double GetUltrasonicInches(UINT8 pingModuleNumber, UINT32 pingChannel, UINT8 echoModuleNumber, UINT32 echoChannel)
{
	Ultrasonic *us = USptr(pingModuleNumber, pingChannel);
	if (us != NULL)
		return us->GetRangeInches();
	else
		return 0.0;
}

/**
 * Get the range in inches from the ultrasonic sensor.
 * @return double Range in inches of the target returned from the ultrasonic sensor. If there is
 * no valid value yet, i.e. at least one measurement hasn't completed, then return 0.
 *
 * @param pingChannel The channel on the digital module for the ping connection
 * @param echoChannel The channel on the digital module for the echo connection
 */
double GetUltrasonicInches(UINT32 pingChannel, UINT32 echoChannel)
{
	return GetUltrasonicInches(SensorBase::GetDefaultDigitalModule(), pingChannel,
								SensorBase::GetDefaultDigitalModule(), echoChannel);
}

/**
 * Get the range in millimeters from the ultrasonic sensor.
 * @return double Range in millimeters of the target returned by the ultrasonic sensor.
 * If there is no valid value yet, i.e. at least one measurement hasn't complted, then return 0.
 *
 * @param pingSlot The slot for the digital module for the ping connection
 * @param pingChannel The channel on the digital module for the ping connection
 * @param echoSlot The slot for the digital module for the echo connection
 * @param echoChannel The channel on the digital module for the echo connection
 */
double GetUltrasonicMM(UINT8 pingModuleNumber, UINT32 pingChannel, UINT8 echoModuleNumber, UINT32 echoChannel)
{
	Ultrasonic *us = USptr(pingModuleNumber, pingChannel);
	if (us != NULL)
		return us->GetRangeMM();
	else
		return 0.0;
}

/**
 * Get the range in millimeters from the ultrasonic sensor.
 * @return double Range in millimeters of the target returned by the ultrasonic sensor.
 * If there is no valid value yet, i.e. at least one measurement hasn't complted, then return 0.
 *
 * @param pingChannel The channel on the digital module for the ping connection
 * @param echoChannel The channel on the digital module for the echo connection
 */
double GetUltrasonicMM(UINT32 pingChannel, UINT32 echoChannel)
{
	return GetUltrasonicMM(SensorBase::GetDefaultDigitalModule(), pingChannel,
							SensorBase::GetDefaultDigitalModule(), echoChannel);
}

/**
 * Free the resources associated with an ultrasonic sensor.
 * Deallocate the Ultrasonic object and free the associated resources.
 *
 * @param pingSlot The slot for the digital module for the ping connection
 * @param pingChannel The channel on the digital module for the ping connection
 * @param echoSlot The slot for the digital module for the echo connection
 * @param echoChannel The channel on the digital module for the echo connection
 */
void DeleteUltrasonic(UINT8 pingModuleNumber, UINT32 pingChannel, UINT8 echoModuleNumber, UINT32 echoChannel)
{
	if (SensorBase::CheckDigitalModule(pingModuleNumber) && SensorBase::CheckDigitalChannel(pingChannel))
	{
		delete ultrasonics[pingModuleNumber - 1][pingChannel - 1];
		ultrasonics[pingModuleNumber - 1][pingChannel - 1] = NULL;
	}
}

/**
 * Free the resources associated with an ultrasonic sensor.
 * Deallocate the Ultrasonic object and free the associated resources.
 *
 * @param pingChannel The channel on the digital module for the ping connection
 * @param echoChannel The channel on the digital module for the echo connection
 */
void DeleteUltrasonic(UINT32 pingChannel, UINT32 echoChannel)
{
	DeleteUltrasonic(SensorBase::GetDefaultDigitalModule(), pingChannel, SensorBase::GetDefaultDigitalModule(), echoChannel);
}

UltrasonicObject CreateUltrasonic(UINT32 pingChannel, UINT32 echoChannel)
{
	return (UltrasonicObject) new Ultrasonic(pingChannel, echoChannel);
}

UltrasonicObject CreateUltrasonic(UINT8 pingModuleNumber, UINT32 pingChannel, UINT8 echoModuleNumber, UINT32 echoChannel)
{
	return (UltrasonicObject) new Ultrasonic(pingModuleNumber, pingChannel, echoModuleNumber, echoChannel);
}

double GetUltrasonicInches(UltrasonicObject o)
{
	return ((Ultrasonic *)o)->GetRangeInches();
}

double GetUltrasonicMM(UltrasonicObject o)
{
	return ((Ultrasonic *)o)->GetRangeMM();
}

void DeleteUltrasonic(UltrasonicObject o)
{
	delete (Ultrasonic *) o;
}

