/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __HiTechnicCompass_h__
#define __HiTechnicCompass_h__

#include "SensorBase.h"
#include "LiveWindow/LiveWindowSendable.h"

class I2C;

/**
 * HiTechnic NXT Compass.
 * 
 * This class alows access to a HiTechnic NXT Compass on an I2C bus.
 * These sensors to not allow changing addresses so you cannot have more
 *   than one on a single bus.
 * 
 * Details on the sensor can be found here:
 *   http://www.hitechnic.com/index.html?lang=en-us&target=d17.html
 * 
 * @todo Implement a calibration method for the sensor.
 */
class HiTechnicCompass : public SensorBase, public LiveWindowSendable
{
public:
	explicit HiTechnicCompass(UINT8 moduleNumber);
	virtual ~HiTechnicCompass();
	float GetAngle();
	
	void UpdateTable();
	void StartLiveWindowMode();
	void StopLiveWindowMode();
	std::string GetSmartDashboardType();
	void InitTable(ITable *subTable);
	ITable * GetTable();

private:
	static const UINT8 kAddress = 0x02;
	static const UINT8 kManufacturerBaseRegister = 0x08;
	static const UINT8 kManufacturerSize = 0x08;
	static const UINT8 kSensorTypeBaseRegister = 0x10;
	static const UINT8 kSensorTypeSize = 0x08;
	static const UINT8 kHeadingRegister = 0x44;

	I2C* m_i2c;
	
	ITable *m_table;
};

#endif

