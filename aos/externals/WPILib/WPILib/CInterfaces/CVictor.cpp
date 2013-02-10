/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "CInterfaces/CVictor.h"
#include "CInterfaces/CPWM.h"
#include "Victor.h"

/**
 * Create an instance of a Victor object (used internally by this module)
 *
 * @param slot The slot that the digital module is plugged into
 * @param channel The PWM channel that the motor is plugged into
 */
static SensorBase *CreateVictorStatic(UINT32 slot, UINT32 channel)
{
	return new Victor(slot, channel);
}


/**
 * Set the PWM value.
 *
 * The PWM value is set using a range of -1.0 to 1.0, appropriately
 * scaling the value for the FPGA.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The PWM channel used for this Victor
 * @param speed The speed value between -1.0 and 1.0 to set.
 */
void SetVictorSpeed(UINT32 slot, UINT32 channel, float speed)
{
	Victor *victor = (Victor *) AllocatePWM(slot, channel, CreateVictorStatic);
	if (victor)	victor->Set(speed);
}

/**
 * Set the PWM value directly to the hardware.
 *
 * Write a raw value to a PWM channel.
 *
 * @param channel The PWM channel used for this Victor
 * @param value Raw PWM value.  Range 0 - 255.
 */
void SetVictorRaw(UINT32 channel, UINT8 value)
{
	Victor *victor = (Victor *) AllocatePWM(channel, CreateVictorStatic);
	if (victor) victor->SetRaw(value);
}

/**
 * Set the PWM value.
 *
 * The PWM value is set using a range of -1.0 to 1.0, appropriately
 * scaling the value for the FPGA.
 *
 * @param channel The PWM channel used for this Victor
 * @param speed The speed value between -1.0 and 1.0 to set.
 */
void SetVictorSpeed(UINT32 channel, float speed)
{
	Victor *victor = (Victor *) AllocatePWM(channel, CreateVictorStatic);
	if (victor) victor->Set(speed);
}

/**
 * Get the PWM value directly from the hardware.
 *
 * Read a raw value from a PWM channel.
 *
 * @param channel The PWM channel used for this Victor
 * @return Raw PWM control value.  Range: 0 - 255.
 */
UINT8 GetVictorRaw(UINT32 channel)
{
	Victor *victor = (Victor *) AllocatePWM(channel, CreateVictorStatic);
	if (victor)
		return victor->GetRaw();
	else
		return 0;
}

/**
 * Set the PWM value directly to the hardware.
 *
 * Write a raw value to a PWM channel.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The PWM channel used for this Victor
 * @param value Raw PWM value.  Range 0 - 255.
 */
void SetVictorRaw(UINT32 slot, UINT32 channel, UINT8 value)
{
	Victor *victor = (Victor *) AllocatePWM(slot, channel, CreateVictorStatic);
	if (victor) victor->SetRaw(value);
}


/**
 * Get the PWM value directly from the hardware.
 *
 * Read a raw value from a PWM channel.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The PWM channel used for this Victor
 * @return Raw PWM control value.  Range: 0 - 255.
 */
UINT8 GetVictorRaw(UINT32 slot, UINT32 channel)
{
	Victor *victor = (Victor *) AllocatePWM(slot, channel, CreateVictorStatic);
	if (victor)
		return victor->GetRaw();
	else
		return 0;
}

/**
 * Delete resources for a Victor
 * Free the underlying object and delete the allocated resources for the Victor
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The PWM channel used for this Victor
 */
void DeleteVictor(UINT32 slot, UINT32 channel)
{
	Victor *victor = (Victor *) AllocatePWM(slot, channel, CreateVictorStatic);
	delete victor;
	DeletePWM(slot, channel);
}

/**
 * Delete resources for a Victor
 * Free the underlying object and delete the allocated resources for the Victor
 *
 * @param channel The PWM channel used for this Victor
 */
void DeleteVictor(UINT32 channel)
{
	DeleteVictor(SensorBase::GetDefaultDigitalModule(), channel);
}

VictorObject CreateVictor(UINT32 slot, UINT32 channel)
{
	return (VictorObject) new Victor(slot, channel);
}

VictorObject CreateVictor(UINT32 channel)
{
	return (VictorObject) new Victor(channel);
}

void DeleteVictor(VictorObject o)
{
	delete (Victor *) o;
}

void SetVictorSpeed(VictorObject o, float speed)
{
	((Victor *)o)->Set(speed);
}

void SetVictorRaw(VictorObject o, UINT8 value)
{
	((Victor *)o)->SetRaw(value);
}

UINT8 GetVictorRaw(VictorObject o)
{
	return ((Victor *)o)->GetRaw();
}

void LoadVictor()
{
}

