/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "CInterfaces/CAnalogChannel.h"
#include "AnalogModule.h"

static bool analogChannelsInitialized = false;
static AnalogChannel
		*analogs[SensorBase::kAnalogModules][SensorBase::kAnalogChannels];

/**
 * Allocate an AnalogChannel object for this set of slot/port
 * @param slot The slot the analog module is plugged into
 * @param channel The channel number on the module for this analog channel object
 */
AnalogChannel *AllocateAnalogChannel(UINT8 moduleNumber, UINT32 channel/*, SensorCreator createObject*/)
{
	if (!analogChannelsInitialized)
	{
		for (unsigned i = 0; i < SensorBase::kAnalogModules; i++)
			for (unsigned j = 0; j < SensorBase::kAnalogChannels; j++)
				analogs[i][j] = NULL;
		analogChannelsInitialized = true;
	}
	if (SensorBase::CheckAnalogModule(moduleNumber) && SensorBase::CheckAnalogChannel(channel))
	{
		if (analogs[moduleNumber - 1][channel - 1] == NULL)
			analogs[moduleNumber - 1][channel - 1] = new AnalogChannel(moduleNumber, channel);
		return analogs[moduleNumber - 1][channel - 1];
	}
	else
		return NULL;
}

/**
 * Get a sample straight from this channel on the module.
 * The sample is a 12-bit value representing the -10V to 10V range of the A/D converter in the module.
 * The units are in A/D converter codes.  Use GetVoltage() to get the analog value in calibrated units.
 *
 * @param slot The slot the analog module is plugged into
 * @param channel the channel for the value being used
 * @return A sample straight from this channel on the module.
 */
INT16 GetAnalogValue(UINT8 moduleNumber, UINT32 channel)
{
	AnalogChannel *analog = AllocateAnalogChannel(moduleNumber, channel);
	if (analog != NULL)
	{
		return analog->GetValue();
	}
	return 0;
}

/**
 * Get a sample from the output of the oversample and average engine for this channel.
 * The sample is 12-bit + the value configured in SetOversampleBits().
 * The value configured in SetAverageBits() will cause this value to be averaged 2**bits number of samples.
 * This is not a sliding window.  The sample will not change until 2**(OversamplBits + AverageBits) samples
 * have been acquired from the module on this channel.
 * Use GetAverageVoltage() to get the analog value in calibrated units.
 *
 * @param slot The slot the analog module is plugged into
 * @param channel the channel for the value being used
 * @return A sample from the oversample and average engine for this channel.
 */
INT32 GetAnalogAverageValue(UINT8 moduleNumber, UINT32 channel)
{
	AnalogChannel *analog = AllocateAnalogChannel(moduleNumber, channel);
	if (analog != NULL)
	{
		return analog->GetAverageValue();
	}
	return 0;
}

/**
 * Get a scaled sample straight from this channel on the module.
 * The value is scaled to units of Volts using the calibrated scaling data from GetLSBWeight() and GetOffset().
 * @param slot The slot the analog module is plugged into
 * @param channel The channel in the module assicated with this analog channel
 * @return A scaled sample straight from this channel on the module.
 */
float GetAnalogVoltage(UINT8 moduleNumber, UINT32 channel)
{
	AnalogChannel *analog = AllocateAnalogChannel(moduleNumber, channel);
	if (analog != NULL)
	{
		return analog->GetVoltage();
	}
	return 0.0;
}

/**
 * Get a scaled sample from the output of the oversample and average engine for this channel.
 * The value is scaled to units of Volts using the calibrated scaling data from GetLSBWeight() and GetOffset().
 * Using oversampling will cause this value to be higher resolution, but it will update more slowly.
 * Using averaging will cause this value to be more stable, but it will update more slowly.
 * @param slot The slot the analog module is plugged into
 * @param channel The channel in the module assicated with this analog channel
 * @return A scaled sample from the output of the oversample and average engine for this channel.
 */
float GetAnalogAverageVoltage(UINT8 moduleNumber, UINT32 channel)
{
	AnalogChannel *analog = AllocateAnalogChannel(moduleNumber, channel);
	if (analog != NULL)
	{
		return analog->GetAverageVoltage();
	}
	return 0.0;
}

/**
 * Set the number of averaging bits.
 * This sets the number of averaging bits. The actual number of averaged samples is 2**bits.
 * Use averaging to improve the stability of your measurement at the expense of sampling rate.
 * The averaging is done automatically in the FPGA.
 *
 * @param slot The slot the analog module is plugged into
 * @param channel The channel in the module assicated with this analog channel
 * @param bits Number of bits of averaging.
 */
void SetAnalogAverageBits(UINT8 moduleNumber, UINT32 channel, UINT32 bits)
{
	AnalogChannel *analog = AllocateAnalogChannel(moduleNumber, channel);
	if (analog != NULL)
	{
		analog->SetAverageBits(bits);
	}
}

/**
 * Get the number of averaging bits previously configured.
 * This gets the number of averaging bits from the FPGA. The actual number of averaged samples is 2**bits.
 * The averaging is done automatically in the FPGA.
 *
 * @param slot The slot the analog module is plugged into
 * @param channel The channel in the module assicated with this analog channel
 * @return Number of bits of averaging previously configured.
 */
UINT32 GetAnalogAverageBits(UINT8 moduleNumber, UINT32 channel)
{
	AnalogChannel *analog = AllocateAnalogChannel(moduleNumber, channel);
	if (analog != NULL)
	{
		return analog->GetAverageBits();
	}
	return 0;
}

/**
 * Set the number of oversample bits.
 * This sets the number of oversample bits. The actual number of oversampled values is 2**bits.
 * Use oversampling to improve the resolution of your measurements at the expense of sampling rate.
 * The oversampling is done automatically in the FPGA.
 *
 * @param slot The slot the analog module is plugged into
 * @param channel The channel in the module assicated with this analog channel
 * @param bits Number of bits of oversampling.
 */
void SetAnalogOversampleBits(UINT8 moduleNumber, UINT32 channel, UINT32 bits)
{
	AnalogChannel *analog = AllocateAnalogChannel(moduleNumber, channel);
	if (analog != NULL)
	{
		analog->SetOversampleBits(bits);
	}
}

/**
 * Get the number of oversample bits previously configured.
 * This gets the number of oversample bits from the FPGA. The actual number of oversampled values is
 * 2**bits. The oversampling is done automatically in the FPGA.
 *
 * @param slot The slot the analog module is plugged into
 * @param channel The channel in the module assicated with this analog channel
 * @return Number of bits of oversampling previously configured.
 */
UINT32 GetAnalogOversampleBits(UINT8 moduleNumber, UINT32 channel)
{
	AnalogChannel *analog = AllocateAnalogChannel(moduleNumber, channel);
	if (analog != NULL)
	{
		return analog->GetOversampleBits();
	}
	return 0;
}

/**
 * Get a sample straight from this channel on the module.
 * The sample is a 12-bit value representing the -10V to 10V range of the A/D converter in the module.
 * The units are in A/D converter codes.  Use GetVoltage() to get the analog value in calibrated units.

 * @param channel The channel in the module assicated with this analog channel
 * @return A sample straight from this channel on the module.
 */
INT16 GetAnalogValue(UINT32 channel)
{
	AnalogChannel *analog = AllocateAnalogChannel(AnalogModule::GetDefaultAnalogModule(), channel);
	if (analog != NULL)
	{
		return analog->GetValue();
	}
	return 0;
}

/**
 * Get a sample from the output of the oversample and average engine for this channel.
 * The sample is 12-bit + the value configured in SetOversampleBits().
 * The value configured in SetAverageBits() will cause this value to be averaged 2**bits number of samples.
 * This is not a sliding window.  The sample will not change until 2**(OversamplBits + AverageBits) samples
 * have been acquired from the module on this channel.
 * Use GetAverageVoltage() to get the analog value in calibrated units.

 * @param channel The channel in the module assicated with this analog channel
 * @return A sample from the oversample and average engine for this channel.
 */
INT32 GetAnalogAverageValue(UINT32 channel)
{
	AnalogChannel *analog = AllocateAnalogChannel(AnalogModule::GetDefaultAnalogModule(), channel);
	if (analog != NULL)
	{
		return analog->GetAverageValue();
	}
	return 0;
}

/**
 * Get a scaled sample straight from this channel on the module.
 * The value is scaled to units of Volts using the calibrated scaling data from GetLSBWeight() and GetOffset().

 * @param channel The channel in the module assicated with this analog channel
 * @return A scaled sample straight from this channel on the module.
 */
float GetAnalogVoltage(UINT32 channel)
{
	AnalogChannel *analog = AllocateAnalogChannel(AnalogModule::GetDefaultAnalogModule(), channel);
	if (analog != NULL)
	{
		return analog->GetVoltage();
	}
	return 0.0;
}

/**
 * Get a scaled sample from the output of the oversample and average engine for this channel.
 * The value is scaled to units of Volts using the calibrated scaling data from GetLSBWeight() and GetOffset().
 * Using oversampling will cause this value to be higher resolution, but it will update more slowly.
 * Using averaging will cause this value to be more stable, but it will update more slowly.

 * @param channel The channel in the module assicated with this analog channel
 * @return A scaled sample from the output of the oversample and average engine for this channel.
 */
float GetAnalogAverageVoltage(UINT32 channel)
{
	AnalogChannel *analog = AllocateAnalogChannel(AnalogModule::GetDefaultAnalogModule(), channel);
	if (analog != NULL)
	{
		return analog->GetAverageVoltage();
	}
	return 0.0;
}

/**
 * Set the number of averaging bits.
 * This sets the number of averaging bits. The actual number of averaged samples is 2**bits.
 * Use averaging to improve the stability of your measurement at the expense of sampling rate.
 * The averaging is done automatically in the FPGA.
 *
 * @param channel The channel in the module assicated with this analog channel
 * @param bits Number of bits of averaging.
 */
void SetAnalogAverageBits(UINT32 channel, UINT32 bits)
{
	AnalogChannel *analog = AllocateAnalogChannel(AnalogModule::GetDefaultAnalogModule(), channel);
	if (analog != NULL)
	{
		analog->SetAverageBits(bits);
	}
}

/**
 * Get the number of averaging bits previously configured.
 * This gets the number of averaging bits from the FPGA. The actual number of averaged samples is 2**bits.
 * The averaging is done automatically in the FPGA.
 *
 * @param channel The channel in the module assicated with this analog channel
 * @return Number of bits of averaging previously configured.
 */
UINT32 GetAnalogAverageBits(UINT32 channel)
{
	AnalogChannel *analog = AllocateAnalogChannel(AnalogModule::GetDefaultAnalogModule(), channel);
	if (analog != NULL)
	{
		return analog->GetAverageBits();
	}
	return 0;
}

/**
 * Set the number of oversample bits.
 * This sets the number of oversample bits. The actual number of oversampled values is 2**bits.
 * Use oversampling to improve the resolution of your measurements at the expense of sampling rate.
 * The oversampling is done automatically in the FPGA.
 *
 * @param channel The channel in the module assicated with this analog channel
 * @param bits Number of bits of oversampling.
 */
void SetAnalogOversampleBits(UINT32 channel, UINT32 bits)
{
	AnalogChannel *analog = AllocateAnalogChannel(AnalogModule::GetDefaultAnalogModule(), channel);
	if (analog != NULL)
	{
		analog->GetOversampleBits();
	}
}

/**
 * Get the number of oversample bits previously configured.
 * This gets the number of oversample bits from the FPGA. The actual number of oversampled values is
 * 2**bits. The oversampling is done automatically in the FPGA.
 *
 * @param channel The channel in the module assicated with this analog channel
 * @return Number of bits of oversampling previously configured.
 */
UINT32 GetAnalogOversampleBits(UINT32 channel)
{
	AnalogChannel *analog = AllocateAnalogChannel(AnalogModule::GetDefaultAnalogModule(), channel);
	if (analog != NULL)
	{
		return analog->GetOversampleBits();
	}
	return 0;
}

/**
 * Delete the resources associated with this AnalogChannel
 * The underlying object and the port reservations are deleted for this analog channel.

 * @param slot The slot the analog module is plugged into
 * @param channel The channel in the module assicated with this analog channel
 */
void DeleteAnalogChannel(UINT8 moduleNumber, UINT32 channel)
{
	if (SensorBase::CheckAnalogModule(moduleNumber) && SensorBase::CheckAnalogChannel(channel))
	{
		delete analogs[moduleNumber - 1][channel - 1];
		analogs[moduleNumber - 1][channel - 1] = NULL;
	}
}

/**
 * Delete the resources associated with this AnalogChannel
 * The underlying object and the port reservations are deleted for this analog channel.

 * @param channel The channel in the module assicated with this analog channel
 */
void DeleteAnalogChannel(UINT32 channel)
{
	DeleteAnalogChannel(SensorBase::GetDefaultAnalogModule(), channel);
}

/**
 * Alternative C Interface
 */

AnalogChannelObject CreateAnalogChannel(UINT32 module, UINT32 channel)
{
	return (AnalogChannelObject) new AnalogChannel(module, channel);
}

AnalogChannelObject CreateAnalogChannel(UINT32 channel)
{
	return (AnalogChannelObject) new AnalogChannel(channel);
}

INT16 GetAnalogValue(AnalogChannelObject o)
{
	return ((AnalogChannel *)o )->GetValue();
}

INT32 GetAnalogAverageValue(AnalogChannelObject o)
{
	return ((AnalogChannel *)o )->GetAverageValue();
}

float GetAnalogVoltage(AnalogChannelObject o)
{
	return ((AnalogChannel *)o )->GetVoltage();
}

float GetAnalogAverageVoltage(AnalogChannelObject o)
{
	return ((AnalogChannel *)o )->GetAverageVoltage();
}

void SetAnalogAverageBits(AnalogChannelObject o, UINT32 bits)
{
	((AnalogChannel *)o )->SetAverageBits(bits);
}

UINT32 GetAnalogAverageBits(AnalogChannelObject o)
{
	return ((AnalogChannel *)o )->GetAverageBits();
}

void SetAnalogOversampleBits(AnalogChannelObject o, UINT32 bits)
{
	((AnalogChannel *)o )->SetOversampleBits(bits);
}

UINT32 GetAnalogOversampleBits(AnalogChannelObject o)
{
	return ((AnalogChannel *)o )->GetOversampleBits();
}

void DeleteAnalogChannel(AnalogChannelObject o)
{
	delete (AnalogChannel *)o;
}
