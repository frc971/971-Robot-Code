/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef PWM_H_
#define PWM_H_

#include "SensorBase.h"
#include "LiveWindow/LiveWindowSendable.h"
#include "tables/ITableListener.h"

class DigitalModule;

/**
 * Class implements the PWM generation in the FPGA.
 * 
 * The values supplied as arguments for PWM outputs range from -1.0 to 1.0. They are mapped
 * to the hardware dependent values, in this case 0-255 for the FPGA.
 * Changes are immediately sent to the FPGA, and the update occurs at the next
 * FPGA cycle. There is no delay.
 * 
 * As of revision 0.1.10 of the FPGA, the FPGA interprets the 0-255 values as follows:
 *   - 255 = full "forward"
 *   - 254 to 129 = linear scaling from "full forward" to "center"
 *   - 128 = center value
 *   - 127 to 2 = linear scaling from "center" to "full reverse"
 *   - 1 = full "reverse"
 *   - 0 = disabled (i.e. PWM output is held low)
 */
class PWM : public SensorBase, public ITableListener, public LiveWindowSendable
{
	friend class DigitalModule;
public:
	typedef enum {kPeriodMultiplier_1X = 1, kPeriodMultiplier_2X = 2, kPeriodMultiplier_4X = 4} PeriodMultiplier;

	explicit PWM(UINT32 channel);
	PWM(UINT8 moduleNumber, UINT32 channel);
	virtual ~PWM();
	virtual void SetRaw(UINT8 value);
	virtual UINT8 GetRaw();
	void SetPeriodMultiplier(PeriodMultiplier mult);
	void EnableDeadbandElimination(bool eliminateDeadband);
	void SetBounds(INT32 max, INT32 deadbandMax, INT32 center, INT32 deadbandMin, INT32 min);
	UINT32 GetChannel() {return m_channel;}
	UINT32 GetModuleNumber();

protected:
	/**
	 * kDefaultPwmPeriod is "ticks" where each tick is 6.525us
	 * 
	 * - 20ms periods (50 Hz) are the "safest" setting in that this works for all devices
	 * - 20ms periods seem to be desirable for Vex Motors
	 * - 20ms periods are the specified period for HS-322HD servos, but work reliably down
	 *      to 10.0 ms; starting at about 8.5ms, the servo sometimes hums and get hot;
	 *      by 5.0ms the hum is nearly continuous
	 * - 10ms periods work well for Victor 884
	 * - 5ms periods allows higher update rates for Luminary Micro Jaguar speed controllers.
	 *      Due to the shipping firmware on the Jaguar, we can't run the update period less
	 *      than 5.05 ms.
	 * 
	 * kDefaultPwmPeriod is the 1x period (5.05 ms).  In hardware, the period scaling is implemented as an
	 * output squelch to get longer periods for old devices.
	 * 
	 * Set to 5.05 ms period / 6.525us clock = 774
	 */
	static const UINT32 kDefaultPwmPeriod = 774;

	/**
	 * kDefaultMinPwmHigh is "ticks" where each tick is 6.525us
	 * 
	 * - There are 128 pwm values less than the center, so...
	 * - The minimum output pulse length is 1.5ms - 128 * 6.525us = 0.665ms
	 * - 0.665ms / 6.525us per tick = 102
	 */
	static const UINT32 kDefaultMinPwmHigh = 102;

	static const INT32 kPwmDisabled = 0;

	virtual void SetPosition(float pos);
	virtual float GetPosition();
	virtual void SetSpeed(float speed);
	virtual float GetSpeed();

	bool m_eliminateDeadband;
	INT32 m_maxPwm;
	INT32 m_deadbandMaxPwm;
	INT32 m_centerPwm;
	INT32 m_deadbandMinPwm;
	INT32 m_minPwm;
	
	void ValueChanged(ITable* source, const std::string& key, EntryValue value, bool isNew);
	void UpdateTable();
	void StartLiveWindowMode();
	void StopLiveWindowMode();
	std::string GetSmartDashboardType();
	void InitTable(ITable *subTable);
	ITable * GetTable();
	
	ITable *m_table;

private:
	void InitPWM(UINT8 moduleNumber, UINT32 channel);
	UINT32 m_channel;
	DigitalModule *m_module;
	INT32 GetMaxPositivePwm() { return m_maxPwm; };
	INT32 GetMinPositivePwm() { return m_eliminateDeadband ? m_deadbandMaxPwm : m_centerPwm + 1; };
	INT32 GetCenterPwm() { return m_centerPwm; };
	INT32 GetMaxNegativePwm() { return m_eliminateDeadband ? m_deadbandMinPwm : m_centerPwm - 1; };
	INT32 GetMinNegativePwm() { return m_minPwm; };
	INT32 GetPositiveScaleFactor() {return GetMaxPositivePwm() - GetMinPositivePwm();} ///< The scale for positive speeds.
	INT32 GetNegativeScaleFactor() {return GetMaxNegativePwm() - GetMinNegativePwm();} ///< The scale for negative speeds.
	INT32 GetFullRangeScaleFactor() {return GetMaxPositivePwm() - GetMinNegativePwm();} ///< The scale for positions.
};

#endif
