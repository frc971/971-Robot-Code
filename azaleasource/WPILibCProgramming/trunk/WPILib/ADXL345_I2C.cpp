/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "ADXL345_I2C.h"
#include "DigitalModule.h"
#include "NetworkCommunication/UsageReporting.h"
#include "I2C.h"

const UINT8 ADXL345_I2C::kAddress;
const UINT8 ADXL345_I2C::kPowerCtlRegister;
const UINT8 ADXL345_I2C::kDataFormatRegister;
const UINT8 ADXL345_I2C::kDataRegister;
const double ADXL345_I2C::kGsPerLSB;

/**
 * Constructor.
 * 
 * @param moduleNumber The digital module that the sensor is plugged into (1 or 2).
 * @param range The range (+ or -) that the accelerometer will measure.
 */
ADXL345_I2C::ADXL345_I2C(UINT8 moduleNumber, ADXL345_I2C::DataFormat_Range range)
	: m_i2c (NULL)
{
	DigitalModule *module = DigitalModule::GetInstance(moduleNumber);
	if (module)
	{
		m_i2c = module->GetI2C(kAddress);

		// Turn on the measurements
		m_i2c->Write(kPowerCtlRegister, kPowerCtl_Measure);
		// Specify the data format to read
		m_i2c->Write(kDataFormatRegister, kDataFormat_FullRes | (UINT8)range);

		nUsageReporting::report(nUsageReporting::kResourceType_ADXL345, nUsageReporting::kADXL345_I2C, moduleNumber - 1);
	}
}

/**
 * Destructor.
 */
ADXL345_I2C::~ADXL345_I2C()
{
	delete m_i2c;
	m_i2c = NULL;
}

/**
 * Get the acceleration of one axis in Gs.
 * 
 * @param axis The axis to read from.
 * @return Acceleration of the ADXL345 in Gs.
 */
double ADXL345_I2C::GetAcceleration(ADXL345_I2C::Axes axis)
{
	INT16 rawAccel = 0;
	if(m_i2c)
	{
		m_i2c->Read(kDataRegister + (UINT8)axis, sizeof(rawAccel), (UINT8 *)&rawAccel);

		// Sensor is little endian... swap bytes
		rawAccel = ((rawAccel >> 8) & 0xFF) | (rawAccel << 8);
	}
	return rawAccel * kGsPerLSB;
}

/**
 * Get the acceleration of all axes in Gs.
 * 
 * @return Acceleration measured on all axes of the ADXL345 in Gs.
 */
ADXL345_I2C::AllAxes ADXL345_I2C::GetAccelerations()
{
	AllAxes data = {0.0};
	INT16 rawData[3];
	if (m_i2c)
	{
		m_i2c->Read(kDataRegister, sizeof(rawData), (UINT8*)rawData);

		// Sensor is little endian... swap bytes
		for (INT32 i=0; i<3; i++)
		{
			rawData[i] = ((rawData[i] >> 8) & 0xFF) | (rawData[i] << 8);
		}

		data.XAxis = rawData[0] * kGsPerLSB;
		data.YAxis = rawData[1] * kGsPerLSB;
		data.ZAxis = rawData[2] * kGsPerLSB;
	}
	return data;
}

