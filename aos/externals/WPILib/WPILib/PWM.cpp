/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "PWM.h"

#include "DigitalModule.h"
#include "NetworkCommunication/UsageReporting.h"
#include "Resource.h"
#include "Utility.h"
#include "WPIErrors.h"

const UINT32 PWM::kDefaultPwmPeriod;
const UINT32 PWM::kDefaultMinPwmHigh;
const INT32 PWM::kPwmDisabled;
static Resource *allocated = NULL;

/**
 * Initialize PWMs given an module and channel.
 * 
 * This method is private and is the common path for all the constructors for creating PWM
 * instances. Checks module and channel value ranges and allocates the appropriate channel.
 * The allocation is only done to help users ensure that they don't double assign channels.
 */
void PWM::InitPWM(UINT8 moduleNumber, UINT32 channel)
{
	char buf[64];
	Resource::CreateResourceObject(&allocated, tDIO::kNumSystems * kPwmChannels);
	if (!CheckPWMModule(moduleNumber))
	{
		snprintf(buf, 64, "Digital Module %d", moduleNumber);
		wpi_setWPIErrorWithContext(ModuleIndexOutOfRange, buf);
		return;
	}
	if (!CheckPWMChannel(channel))
	{
		snprintf(buf, 64, "PWM Channel %d", channel);
		wpi_setWPIErrorWithContext(ChannelIndexOutOfRange, buf);
		return;
	}

	snprintf(buf, 64, "PWM %d (Module: %d)", channel, moduleNumber);
	if (allocated->Allocate((moduleNumber - 1) * kPwmChannels + channel - 1, buf) == ~0ul)
	{
		CloneError(allocated);
		return;
	}
	m_channel = channel;
	m_module = DigitalModule::GetInstance(moduleNumber);
	m_module->SetPWM(m_channel, kPwmDisabled);
	m_eliminateDeadband = false;

	nUsageReporting::report(nUsageReporting::kResourceType_PWM, channel, moduleNumber - 1);
}

/**
 * Allocate a PWM given a module and channel.
 * Allocate a PWM using a module and channel number.
 * 
 * @param moduleNumber The digital module (1 or 2).
 * @param channel The PWM channel on the digital module (1..10).
 */
PWM::PWM(UINT8 moduleNumber, UINT32 channel)
	: m_module(NULL)
{
	InitPWM(moduleNumber, channel);
}

/**
 * Allocate a PWM in the default module given a channel.
 * 
 * Using a default module allocate a PWM given the channel number.  The default module is the first
 * slot numerically in the cRIO chassis.
 * 
 * @param channel The PWM channel on the digital module.
 */
PWM::PWM(UINT32 channel)
	: m_module(NULL)
{
	InitPWM(GetDefaultDigitalModule(), channel);
}

/**
 * Free the PWM channel.
 * 
 * Free the resource associated with the PWM channel and set the value to 0.
 */
PWM::~PWM()
{
	if (m_module)
	{
		m_module->SetPWM(m_channel, kPwmDisabled);
		allocated->Free((m_module->GetNumber() - 1) * kPwmChannels + m_channel - 1);
	}
}

/**
 * Optionally eliminate the deadband from a speed controller.
 * @param eliminateDeadband If true, set the motor curve on the Jaguar to eliminate
 * the deadband in the middle of the range. Otherwise, keep the full range without
 * modifying any values.
 */
void PWM::EnableDeadbandElimination(bool eliminateDeadband)
{
	if (StatusIsFatal()) return;
	m_eliminateDeadband = eliminateDeadband;
}

/**
 * Set the bounds on the PWM values.
 * This sets the bounds on the PWM values for a particular each type of controller. The values
 * determine the upper and lower speeds as well as the deadband bracket.
 * @param max The Minimum pwm value
 * @param deadbandMax The high end of the deadband range
 * @param center The center speed (off)
 * @param deadbandMin The low end of the deadband range
 * @param min The minimum pwm value
 */
void PWM::SetBounds(INT32 max, INT32 deadbandMax, INT32 center, INT32 deadbandMin, INT32 min)
{
	if (StatusIsFatal()) return;
	m_maxPwm = max;
	m_deadbandMaxPwm = deadbandMax;
	m_centerPwm = center;
	m_deadbandMinPwm = deadbandMin;
	m_minPwm = min;
}

UINT32 PWM::GetModuleNumber()
{
	return m_module->GetNumber();
}

/**
 * Set the PWM value based on a position.
 * 
 * This is intended to be used by servos.
 * 
 * @pre SetMaxPositivePwm() called.
 * @pre SetMinNegativePwm() called.
 * 
 * @param pos The position to set the servo between 0.0 and 1.0.
 */
void PWM::SetPosition(float pos)
{
	if (StatusIsFatal()) return;
	if (pos < 0.0)
	{
		pos = 0.0;
	}
	else if (pos > 1.0)
	{
		pos = 1.0;
	}

	INT32 rawValue;
	// note, need to perform the multiplication below as floating point before converting to int
	rawValue = (INT32)( (pos * (float) GetFullRangeScaleFactor()) + GetMinNegativePwm());

	wpi_assert((rawValue >= GetMinNegativePwm()) && (rawValue <= GetMaxPositivePwm()));
	wpi_assert(rawValue != kPwmDisabled);

	// send the computed pwm value to the FPGA
	SetRaw((UINT8)rawValue);
}

/**
 * Get the PWM value in terms of a position.
 * 
 * This is intended to be used by servos.
 * 
 * @pre SetMaxPositivePwm() called.
 * @pre SetMinNegativePwm() called.
 * 
 * @return The position the servo is set to between 0.0 and 1.0.
 */
float PWM::GetPosition()
{
	if (StatusIsFatal()) return 0.0;
	INT32 value = GetRaw();
	if (value < GetMinNegativePwm())
	{
		return 0.0;
	}
	else if (value > GetMaxPositivePwm())
	{
		return 1.0;
	}
	else
	{
		return (float)(value - GetMinNegativePwm()) / (float)GetFullRangeScaleFactor();
	}
}

/**
 * Set the PWM value based on a speed.
 * 
 * This is intended to be used by speed controllers.
 * 
 * @pre SetMaxPositivePwm() called.
 * @pre SetMinPositivePwm() called.
 * @pre SetCenterPwm() called.
 * @pre SetMaxNegativePwm() called.
 * @pre SetMinNegativePwm() called.
 * 
 * @param speed The speed to set the speed controller between -1.0 and 1.0.
 */
void PWM::SetSpeed(float speed)
{
	if (StatusIsFatal()) return;
	// clamp speed to be in the range 1.0 >= speed >= -1.0
	if (speed < -1.0)
	{
		speed = -1.0;
	}
	else if (speed > 1.0)
	{
		speed = 1.0;
	}

	// calculate the desired output pwm value by scaling the speed appropriately
	INT32 rawValue;
	if (speed == 0.0)
	{
		rawValue = GetCenterPwm();
	}
	else if (speed > 0.0)
	{
		rawValue = (INT32)(speed * ((float)GetPositiveScaleFactor()) +
									((float) GetMinPositivePwm()) + 0.5);
	}
	else
	{
		rawValue = (INT32)(speed * ((float)GetNegativeScaleFactor()) +
									((float) GetMaxNegativePwm()) + 0.5);
	}

	// the above should result in a pwm_value in the valid range
	wpi_assert((rawValue >= GetMinNegativePwm()) && (rawValue <= GetMaxPositivePwm()));
	wpi_assert(rawValue != kPwmDisabled);

	// send the computed pwm value to the FPGA
	SetRaw((UINT8)rawValue);
}

/**
 * Get the PWM value in terms of speed.
 * 
 * This is intended to be used by speed controllers.
 * 
 * @pre SetMaxPositivePwm() called.
 * @pre SetMinPositivePwm() called.
 * @pre SetMaxNegativePwm() called.
 * @pre SetMinNegativePwm() called.
 * 
 * @return The most recently set speed between -1.0 and 1.0.
 */
float PWM::GetSpeed()
{
	if (StatusIsFatal()) return 0.0;
	INT32 value = GetRaw();
	if (value > GetMaxPositivePwm())
	{
		return 1.0;
	}
	else if (value < GetMinNegativePwm())
	{
		return -1.0;
	}
	else if (value > GetMinPositivePwm())
	{
		return (float)(value - GetMinPositivePwm()) / (float)GetPositiveScaleFactor();
	}
	else if (value < GetMaxNegativePwm())
	{
		return (float)(value - GetMaxNegativePwm()) / (float)GetNegativeScaleFactor();
	}
	else
	{
		return 0.0;
	}
}

/**
 * Set the PWM value directly to the hardware.
 * 
 * Write a raw value to a PWM channel.
 * 
 * @param value Raw PWM value.  Range 0 - 255.
 */
void PWM::SetRaw(UINT8 value)
{
	if (StatusIsFatal()) return;
	m_module->SetPWM(m_channel, value);
}

/**
 * Get the PWM value directly from the hardware.
 * 
 * Read a raw value from a PWM channel.
 * 
 * @return Raw PWM control value.  Range: 0 - 255.
 */
UINT8 PWM::GetRaw()
{
	if (StatusIsFatal()) return 0;
	return m_module->GetPWM(m_channel);
}

/**
 * Slow down the PWM signal for old devices.
 * 
 * @param mult The period multiplier to apply to this channel
 */
void PWM::SetPeriodMultiplier(PeriodMultiplier mult)
{
	if (StatusIsFatal()) return;
	switch(mult)
	{
	case kPeriodMultiplier_4X:
		m_module->SetPWMPeriodScale(m_channel, 3); // Squelch 3 out of 4 outputs
		break;
	case kPeriodMultiplier_2X:
		m_module->SetPWMPeriodScale(m_channel, 1); // Squelch 1 out of 2 outputs
		break;
	case kPeriodMultiplier_1X:
		m_module->SetPWMPeriodScale(m_channel, 0); // Don't squelch any outputs
		break;
	default:
		wpi_assert(false);
	}
}


void PWM::ValueChanged(ITable* source, const std::string& key, EntryValue value, bool isNew) {
	SetSpeed(value.f);
}

void PWM::UpdateTable() {
	if (m_table != NULL) {
		m_table->PutNumber("Value", GetSpeed());
	}
}

void PWM::StartLiveWindowMode() {
	m_table->AddTableListener("Value", this, true);
}

void PWM::StopLiveWindowMode() {
	SetSpeed(0);
	m_table->RemoveTableListener(this);
}

std::string PWM::GetSmartDashboardType() {
	return "Speed Controller";
}

void PWM::InitTable(ITable *subTable) {
	m_table = subTable;
	UpdateTable();
}

ITable * PWM::GetTable() {
	return m_table;
}

