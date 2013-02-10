/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "Victor.h"

#include "DigitalModule.h"
#include "NetworkCommunication/UsageReporting.h"

/**
 * Common initialization code called by all constructors.
 * 
 * Note that the Victor uses the following bounds for PWM values.  These values were determined
 * empirically through experimentation during the 2008 beta testing of the new control system.
 * Testing during the beta period revealed a significant amount of variation between Victors.
 * The values below are chosen to ensure that teams using the default values should be able to
 * get "full power" with the maximum and minimum values.  For better performance, teams may wish
 * to measure these values on their own Victors and set the bounds to the particular values
 * measured for the actual Victors they were be using. 
 *   - 210 = full "forward"
 *   - 138 = the "high end" of the deadband range
 *   - 132 = center of the deadband range (off)
 *   - 126 = the "low end" of the deadband range
 *   - 56 = full "reverse"
 */
void Victor::InitVictor()
{
	// TODO: compute the appropriate values based on digital loop timing
	SetBounds(210, 138, 132, 126, 56);
	SetPeriodMultiplier(kPeriodMultiplier_2X);
	SetRaw(m_centerPwm);

	nUsageReporting::report(nUsageReporting::kResourceType_Victor, GetChannel(), GetModuleNumber() - 1);
}

/**
 * Constructor that assumes the default digital module.
 * 
 * @param channel The PWM channel on the digital module that the Victor is attached to.
 */
Victor::Victor(UINT32 channel) : SafePWM(channel)
{
	InitVictor();
}

/**
 * Constructor that specifies the digital module.
 * 
 * @param moduleNumber The digital module (1 or 2).
 * @param channel The PWM channel on the digital module that the Victor is attached to (1..10).
 */
Victor::Victor(UINT8 moduleNumber, UINT32 channel) : SafePWM(moduleNumber, channel)
{
	InitVictor();
}

Victor::~Victor()
{
}

/**
 * Set the PWM value.  
 * 
 * The PWM value is set using a range of -1.0 to 1.0, appropriately
 * scaling the value for the FPGA.
 * 
 * @param speed The speed value between -1.0 and 1.0 to set.
 * @param syncGroup Unused interface.
 */
void Victor::Set(float speed, UINT8 syncGroup)
{
	SetSpeed(speed);
}

/**
 * Get the recently set value of the PWM.
 * 
 * @return The most recently set value for the PWM between -1.0 and 1.0.
 */
float Victor::Get()
{
	return GetSpeed();
}

/**
 * Common interface for disabling a motor.
 */
void Victor::Disable()
{
	SetRaw(kPwmDisabled);
}

/**
 * Write out the PID value as seen in the PIDOutput base object.
 * 
 * @param output Write out the PWM value as was found in the PIDController
 */
void Victor::PIDWrite(float output) 
{
	Set(output);
}

