/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "Encoder.h"
#include "SensorBase.h"
#include "DigitalModule.h"
#include "CInterfaces/CEncoder.h"

static Encoder* encoders[SensorBase::kDigitalModules][SensorBase::kDigitalChannels];
static bool initialized = false;

/**
 * Allocate the resources associated with this encoder.
 * Allocate an Encoder object and cache the value in the associated table to find it
 * in the future.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
static Encoder *AllocateEncoder(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel)
{
	Encoder *encoder;
	if (!initialized)
	{
		for (unsigned slot = 0; slot < SensorBase::kDigitalModules; slot++)
			for (unsigned channel = 0; channel < SensorBase::kDigitalChannels; channel++)
				encoders[slot][channel] = NULL;
		initialized = true;
	}
	// check if the channel and slots are valid values
	if (!SensorBase::CheckDigitalModule(amoduleNumber)
			|| !SensorBase::CheckDigitalChannel(aChannel)
			|| !SensorBase::CheckDigitalModule(bmoduleNumber)
			|| !SensorBase::CheckDigitalChannel(bChannel)) return NULL;
	// check if nothing has been allocated to that pair of channels
	if (encoders[amoduleNumber - 1][aChannel - 1] == NULL
				&& encoders[bmoduleNumber - 1][bChannel - 1] == NULL)
	{
		// allocate an encoder and put it into both slots
		encoder = new Encoder(amoduleNumber, aChannel, bmoduleNumber, bChannel);
		encoders[amoduleNumber - 1][aChannel - 1] = encoder;
		encoders[bmoduleNumber - 1][bChannel - 1] = encoder;
		return encoder;
	}
	// if there was the same encoder object allocated to both channels, return it
	if ((encoder = encoders[amoduleNumber - 1][aChannel - 1]) ==
				 encoders[bmoduleNumber - 1][bChannel - 1])
		return encoder;
	// Otherwise, one of the channels is allocated and the other isn't, so this is a
	// resource allocation error.
	return NULL;
}

/**
 * Allocate the resources associated with this encoder.
 * Allocate an Encoder object and cache the value in the associated table to find it
 * in the future.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
static Encoder *AllocateEncoder(UINT32 aChannel, UINT32 bChannel)
{
	return AllocateEncoder(SensorBase::GetDefaultDigitalModule(), aChannel,
							SensorBase::GetDefaultDigitalModule(), bChannel);
}

/**
 * Start the encoder counting.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void StartEncoder(UINT32 aChannel, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
	{
		encoder->Start();
	}
}

/**
 * Start the encoder counting.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void StartEncoder(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(amoduleNumber, aChannel, bmoduleNumber, bChannel);
	if (encoder != NULL)
	{
		encoder->Start();
	}
}

/**
 * Gets the current count.
 * Returns the current count on the Encoder.
 * This method compensates for the decoding type.
 *
 * @deprecated Use GetEncoderDistance() in favor of this method.  This returns unscaled pulses and GetDistance() scales using value from SetEncoderDistancePerPulse().
 *
 * @return Current count from the Encoder.
 * 
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
INT32 GetEncoder(UINT32 aChannel, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		return encoder->Get();
	else
		return 0;
}

/**
 * Gets the current count.
 * Returns the current count on the Encoder.
 * This method compensates for the decoding type.
 * 
 * @deprecated Use GetEncoderDistance() in favor of this method.  This returns unscaled pulses and GetDistance() scales using value from SetEncoderDistancePerPulse().
 *
 * @return Current count from the Encoder.
 * 
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
INT32 GetEncoder(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(amoduleNumber, aChannel, amoduleNumber, bChannel);
	if (encoder != NULL)
		return encoder->Get();
	else
		return 0;
}

/**
 * Reset the count for the encoder object.
 * Resets the count to zero.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void ResetEncoder(UINT32 aChannel, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		encoder->Reset();
}

/**
 * Reset the count for the encoder object.
 * Resets the count to zero.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void ResetEncoder(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(amoduleNumber, aChannel, bmoduleNumber, bChannel);
	if (encoder != NULL)
		encoder->Reset();
}

/**
 * Stops the counting for the encoder object.
 * Stops the counting for the Encoder. It still retains the count, but it doesn't change
 * with pulses until it is started again.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void StopEncoder(UINT32 aChannel, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		encoder->Stop();
}

/**
 * Stops the counting for the encoder object.
 * Stops the counting for the Encoder. It still retains the count, but it doesn't change
 * with pulses until it is started again.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void StopEncoder(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(amoduleNumber, aChannel, bmoduleNumber, bChannel);
	if (encoder != NULL)
		encoder->Stop();
}

/**
 * Returns the period of the most recent pulse.
 * Returns the period of the most recent Encoder pulse in seconds.
 * This method compenstates for the decoding type.
 * 
 * @deprecated Use GetEncoderRate() in favor of this method.  This returns unscaled periods and GetEncoderRate() scales using value from SetEncoderDistancePerPulse().
 *
 * @return Period in seconds of the most recent pulse.
 * 
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
double GetEncoderPeriod(UINT32 aChannel, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		return encoder->GetPeriod();
	else
		return 0.0;
}

/**
 * Returns the period of the most recent pulse.
 * Returns the period of the most recent Encoder pulse in seconds.
 * This method compenstates for the decoding type.
 * 
 * @deprecated Use GetEncoderRate() in favor of this method.  This returns unscaled periods and GetEncoderRate() scales using value from SetEncoderDistancePerPulse().
 *
 * @return Period in seconds of the most recent pulse.
 * 
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
double GetEncoderPeriod(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(amoduleNumber, aChannel, bmoduleNumber, bChannel);
	if (encoder != NULL)
		return encoder->GetPeriod();
	else
		return 0.0;
}


/**
 * Sets the maximum period for stopped detection.
 * Sets the value that represents the maximum period of the Encoder before it will assume
 * that the attached device is stopped. This timeout allows users to determine if the wheels or
 * other shaft has stopped rotating.
 * This method compensates for the decoding type.
 * 
 * @deprecated Use SetEncoderMinRate() in favor of this method.  This takes unscaled periods and SetMinEncoderRate() scales using value from SetEncoderDistancePerPulse().
 * 
 * @param maxPeriod The maximum time between rising and falling edges before the FPGA will
 * report the device stopped. This is expressed in seconds.
 * 
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void SetMaxEncoderPeriod(UINT32 aChannel, UINT32 bChannel, double maxPeriod)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		encoder->SetMaxPeriod(maxPeriod);
}

/**
 * Sets the maximum period for stopped detection.
 * Sets the value that represents the maximum period of the Encoder before it will assume
 * that the attached device is stopped. This timeout allows users to determine if the wheels or
 * other shaft has stopped rotating.
 * This method compensates for the decoding type.
 * 
 * @deprecated Use SetEncoderMinRate() in favor of this method.  This takes unscaled periods and SetMinEncoderRate() scales using value from SetEncoderDistancePerPulse().
 * 
 * @param maxPeriod The maximum time between rising and falling edges before the FPGA will
 * report the device stopped. This is expressed in seconds.
 * 
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void SetMaxEncoderPeriod(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel, double maxPeriod)
{
	Encoder *encoder = AllocateEncoder(amoduleNumber, aChannel, bmoduleNumber, bChannel);
	if (encoder != NULL)
		encoder->SetMaxPeriod(maxPeriod);
}

/**
 * Determine if the encoder is stopped.
 * Using the MaxPeriod value, a boolean is returned that is true if the encoder is considered
 * stopped and false if it is still moving. A stopped encoder is one where the most recent pulse
 * width exceeds the MaxPeriod.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 * @return True if the encoder is considered stopped.
 */
bool GetEncoderStopped(UINT32 aChannel, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		return encoder->GetStopped();
	else
		return false;
}

/**
 * Determine if the encoder is stopped.
 * Using the MaxPeriod value, a boolean is returned that is true if the encoder is considered
 * stopped and false if it is still moving. A stopped encoder is one where the most recent pulse
 * width exceeds the MaxPeriod.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 * @return True if the encoder is considered stopped.
 */
bool GetEncoderStopped(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(amoduleNumber, aChannel, bmoduleNumber, bChannel);
	if (encoder != NULL)
		return encoder->GetStopped();
	else
		return false;
}

/**
 * The last direction the encoder value changed.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 * @return The last direction the encoder value changed.
 */
bool GetEncoderDirection(UINT32 aChannel, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		return encoder->GetDirection();
	else
		return false;
}

/**
 * The last direction the encoder value changed.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 * @return The last direction the encoder value changed.
 */
bool GetEncoderDirection(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(amoduleNumber, aChannel, bmoduleNumber, bChannel);
	if (encoder != NULL)
		return encoder->GetDirection();
	else
		return false;
}

/**
 * Get the distance the robot has driven since the last reset.
 * 
 * @return The distance driven since the last reset as scaled by the value from SetEncoderDistancePerPulse().
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
double GetEncoderDistance(UINT32 aChannel, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		return encoder->GetDistance();
	else
		return 0.0;
}

/**
 * Get the distance the robot has driven since the last reset.
 * 
 * @return The distance driven since the last reset as scaled by the value from SetEncoderDistancePerPulse().
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
  */
double GetEncoderDistance(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(amoduleNumber, aChannel, bmoduleNumber, bChannel);
	if (encoder != NULL)
		return encoder->GetDistance();
	else
		return 0.0;
}

/**
 * Get the current rate of the encoder.
 * Units are distance per second as scaled by the value from SetEncoderDistancePerPulse().
 * 
 * @return The current rate of the encoder.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
double GetEncoderRate(UINT32 aChannel, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		return encoder->GetRate();
	else
		return 0.0;
}

/**
 * Get the current rate of the encoder.
 * Units are distance per second as scaled by the value from SetEncoderDistancePerPulse().
 * 
 * @return The current rate of the encoder.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
  */
double GetEncoderRate(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(amoduleNumber, aChannel, bmoduleNumber, bChannel);
	if (encoder != NULL)
		return encoder->GetRate();
	else
		return 0.0;
}

/**
 * Set the minimum rate of the device before the hardware reports it stopped.
 * 
 * @param minRate The minimum rate.  The units are in distance per second as scaled by the value from SetEncoderDistancePerPulse().
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void SetMinEncoderRate(UINT32 aChannel, UINT32 bChannel, double minRate)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		encoder->SetMinRate(minRate);
}

/**
 * Set the minimum rate of the device before the hardware reports it stopped.
 * 
 * @param minRate The minimum rate.  The units are in distance per second as scaled by the value from SetEncoderDistancePerPulse().
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void SetMinEncoderRate(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel, double minRate)
{
	Encoder *encoder = AllocateEncoder(amoduleNumber, aChannel, bmoduleNumber, bChannel);
	if (encoder != NULL)
		encoder->SetMinRate(minRate);
}

/**
 * Set the distance per pulse for this encoder.
 * This sets the multiplier used to determine the distance driven based on the count value
 * from the encoder.
 * Do not include the decoding type in this scale.  The library already compensates for the decoding type.
 * Set this value based on the encoder's rated Pulses per Revolution and
 * factor in gearing reductions following the encoder shaft.
 * This distance can be in any units you like, linear or angular.
 * 
 * @param distancePerPulse The scale factor that will be used to convert pulses to useful units.
 * 
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void SetEncoderDistancePerPulse(UINT32 aChannel, UINT32 bChannel, double distancePerPulse)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		encoder->SetDistancePerPulse(distancePerPulse);
}

/**
 * Set the distance per pulse for this encoder.
 * This sets the multiplier used to determine the distance driven based on the count value
 * from the encoder.
 * Do not include the decoding type in this scale.  The library already compensates for the decoding type.
 * Set this value based on the encoder's rated Pulses per Revolution and
 * factor in gearing reductions following the encoder shaft.
 * This distance can be in any units you like, linear or angular.
 * 
 * @param distancePerPulse The scale factor that will be used to convert pulses to useful units.
 * 
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void SetEncoderDistancePerPulse(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel, double distancePerPulse)
{
	Encoder *encoder = AllocateEncoder(amoduleNumber, aChannel, bmoduleNumber, bChannel);
	if (encoder != NULL)
		encoder->SetDistancePerPulse(distancePerPulse);
}

/**
 * Set the direction sensing for this encoder.
 * This sets the direction sensing on the encoder so that it could count in the correct
 * software direction regardless of the mounting.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 * @param reverseDirection true if the encoder direction should be reversed
 */
void SetEncoderReverseDirection(UINT32 aChannel, UINT32 bChannel, bool reverseDirection)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		encoder->SetReverseDirection(reverseDirection);
}

/**
 * Set the direction sensing for this encoder.
 * This sets the direction sensing on the encoder so that it couldl count in the correct
 * software direction regardless of the mounting.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 * @param reverseDirection true if the encoder direction should be reversed
 */
void SetEncoderReverseDirection(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel, bool reverseDirection)
{
	Encoder *encoder = AllocateEncoder(amoduleNumber, aChannel, bmoduleNumber, bChannel);
	if (encoder != NULL)
		encoder->SetReverseDirection(reverseDirection);
}

/**
 * Free the resources associated with this encoder.
 * Delete the Encoder object and the entries from the cache for this encoder.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void DeleteEncoder(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel)
{
	if (SensorBase::CheckDigitalModule(amoduleNumber) && SensorBase::CheckDigitalChannel(aChannel) &&
	        SensorBase::CheckDigitalModule(bmoduleNumber) && SensorBase::CheckDigitalChannel(bChannel))
	{
		delete encoders[amoduleNumber - 1][aChannel - 1];
        encoders[amoduleNumber - 1][aChannel - 1] = NULL;
        encoders[bmoduleNumber - 1][bChannel - 1] = NULL;
	}
}

/**
 * Free the resources associated with this encoder.
 * Delete the Encoder object and the entries from the cache for this encoder.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void DeleteEncoder(UINT32 aChannel, UINT32 bChannel)
{
	DeleteEncoder(SensorBase::GetDefaultDigitalModule(), aChannel, SensorBase::GetDefaultDigitalModule(), bChannel);
}

/**
 * Alternate C Interface
 */

EncoderObject CreateEncoder(UINT32 aChannel, UINT32 bChannel)
{
	return (EncoderObject) new Encoder(aChannel, bChannel);
}

EncoderObject CreateEncoder(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel)
{
	return (EncoderObject) new Encoder(amoduleNumber, aChannel, bmoduleNumber, bChannel);
}

void StartEncoder(EncoderObject o)
{
	((Encoder *)o )->Start();
}

INT32 GetEncoder(EncoderObject o)
{
	return ((Encoder *)o )->Get();
}

void ResetEncoder(EncoderObject o)
{
	((Encoder *)o )->Reset();
}

void StopEncoder(EncoderObject o)
{
	((Encoder *)o )->Stop();
}

double GetEncoderPeriod(EncoderObject o)
{
	return ((Encoder *)o )->GetPeriod();
}

void SetMaxEncoderPeriod(EncoderObject o, double maxPeriod)
{
	((Encoder *)o )->SetMaxPeriod(maxPeriod);
}

bool GetEncoderStopped(EncoderObject o)
{
	return ((Encoder *)o )->GetStopped();
}

bool GetEncoderDirection(EncoderObject o)
{
	return ((Encoder *)o )->GetDirection();
}

double GetEncoderDistance(EncoderObject o)
{
	return ((Encoder *)o )->GetDistance();
}

double GetEncoderRate(EncoderObject o)
{
	return ((Encoder *)o )->GetRate();
}

void SetMinEncoderRate(EncoderObject o, double minRate)
{
	((Encoder *)o )->SetMinRate(minRate);
}

void SetEncoderDistancePerPulse(EncoderObject o, double distancePerPulse)
{
	((Encoder *)o )->SetDistancePerPulse(distancePerPulse);
}

void SetEncoderReverseDirection(EncoderObject o, bool reversedDirection)
{
	((Encoder *)o )->SetReverseDirection(reversedDirection);
}

void DeleteEncoder(EncoderObject o)
{
	delete (Encoder *)o;
}
