/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "HiTechnicColorSensor.h"
#include "DigitalModule.h"
#include "I2C.h"
#include "NetworkCommunication/UsageReporting.h"
#include "WPIErrors.h"

const UINT8 HiTechnicColorSensor::kAddress;
const UINT8 HiTechnicColorSensor::kManufacturerBaseRegister;
const UINT8 HiTechnicColorSensor::kManufacturerSize;
const UINT8 HiTechnicColorSensor::kSensorTypeBaseRegister;
const UINT8 HiTechnicColorSensor::kSensorTypeSize;
const UINT8 HiTechnicColorSensor::kModeRegister;
const UINT8 HiTechnicColorSensor::kColorRegister;
const UINT8 HiTechnicColorSensor::kRedRegister;
const UINT8 HiTechnicColorSensor::kGreenRegister;
const UINT8 HiTechnicColorSensor::kBlueRegister;
const UINT8 HiTechnicColorSensor::kRawRedRegister;
const UINT8 HiTechnicColorSensor::kRawGreenRegister;
const UINT8 HiTechnicColorSensor::kRawBlueRegister;

/**
 * Constructor.
 * 
 * @param moduleNumber The digital module that the sensor is plugged into (1 or 2).
 */
HiTechnicColorSensor::HiTechnicColorSensor(UINT8 moduleNumber)
	: m_i2c (NULL)
{
	DigitalModule *module = DigitalModule::GetInstance(moduleNumber);
	m_mode = kActive;
	
	if (module)
	{
		m_i2c = module->GetI2C(kAddress);
	
		// Verify Sensor
		const UINT8 kExpectedManufacturer[] = "HiTechnc";
		const UINT8 kExpectedSensorType[] = "ColorPD ";
		if ( ! m_i2c->VerifySensor(kManufacturerBaseRegister, kManufacturerSize, kExpectedManufacturer) )
		{
			wpi_setWPIError(CompassManufacturerError);
			return;
		}
		if ( ! m_i2c->VerifySensor(kSensorTypeBaseRegister, kSensorTypeSize, kExpectedSensorType) )
		{
			wpi_setWPIError(CompassTypeError);
		}
		
		nUsageReporting::report(nUsageReporting::kResourceType_HiTechnicColorSensor, moduleNumber - 1);
	}
}

/**
 * Destructor.
 */
HiTechnicColorSensor::~HiTechnicColorSensor()
{
	delete m_i2c;
	m_i2c = NULL;
}

/**
 * Get the estimated color.
 *
 * Gets a color estimate from the sensor corresponding to the
 * table found with the sensor or at the following site:
 * http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NCO1038
 *
 * @return The estimated color.
 */
UINT8 HiTechnicColorSensor::GetColor()
{
	UINT8 color = 0;
	
	if(m_mode != kActive)
	{
		SetMode(kActive);
	}
	if (m_i2c)
	{
		m_i2c->Read(kColorRegister, sizeof(color), &color);
	}
	return color;
}

/**
 * Get the Red value.
 *
 * Gets the (0-255) red value from the sensor.
 * 
 * The sensor must be in active mode to access the regular RGB data
 * if the sensor is not in active mode, it will be placed into active
 * mode by this method.
 *
 * @return The Red sensor value.
 */
UINT8 HiTechnicColorSensor::GetRed()
{
	UINT8 red = 0;
	
	if(m_mode != kActive)
	{
		SetMode(kActive);
	}
	if (m_i2c)
	{
		m_i2c->Read(kRedRegister, sizeof(red), &red);
	}
	return red;
}

/**
 * Get the Green value.
 *
 * Gets the(0-255) green value from the sensor.
 * 
 * The sensor must be in active mode to access the regular RGB data
 * if the sensor is not in active mode, it will be placed into active
 * mode by this method.
 * 
 * @return The Green sensor value.
 */
UINT8 HiTechnicColorSensor::GetGreen()
{
	UINT8 green = 0;
	
	if(m_mode != kActive)
	{
		SetMode(kActive);
	}
	if (m_i2c)
	{
		m_i2c->Read(kGreenRegister, sizeof(green), &green);
	}
	return green;
}

/**
 * Get the Blue value.
 *
 * Gets the raw (0-255) blue value from the sensor.
 * 
 * The sensor must be in active mode to access the regular RGB data
 * if the sensor is not in active mode, it will be placed into active
 * mode by this method.
 * 
 * @return The Blue sensor value.
 */
UINT8 HiTechnicColorSensor::GetBlue()
{
	UINT8 blue = 0;
	
	if(m_mode != kActive)
	{
		SetMode(kActive);
	}
	if (m_i2c)
	{
		m_i2c->Read(kBlueRegister, sizeof(blue), &blue);
	}
	return blue;
}

/**
 * Get the value of all three colors from a single sensor reading.
 * Using this method ensures that all three values come from the
 * same sensor reading, using the individual color methods provides
 * no such guarantee.
 * 
 * The sensor must be in active mode to access the regular RGB data.
 * If the sensor is not in active mode, it will be placed into active
 * mode by this method.
 * 
 * @return RGB object with the three color values
 */
HiTechnicColorSensor::RGB HiTechnicColorSensor::GetRGB()
{
	UINT8 colors[3] = {0,0,0};
	RGB result;
	
	if(m_mode != kActive)
	{
		SetMode(kActive);
	}
	if(m_i2c)
	{
		m_i2c->Read(kRawRedRegister, sizeof(colors), (UINT8*)&colors);
	}
	
	result.red = colors[0];
	result.green = colors[1];
	result.blue = colors[2];
	
	return result;
}

/**
 * Get the Raw Red value.
 *
 * Gets the (0-65536) raw red value from the sensor.
 * 
 * The sensor must be in raw or passive mode to access the regular RGB data
 * if the sensor is not in raw or passive mode, it will be placed into raw
 * mode by this method.
 *
 * @return The Raw Red sensor value.
 */
UINT16 HiTechnicColorSensor::GetRawRed()
{
	UINT16 rawRed = 0;
	
	if(m_mode == kActive)
	{
		SetMode(kRaw);
	}
	if (m_i2c)
	{
		m_i2c->Read(kRawRedRegister, sizeof(rawRed), (UINT8 *)&rawRed);
	}
	return rawRed;
}

/**
   * Get the Raw Green value.
   *
   * Gets the (0-65536) raw green value from the sensor.
   * 
   * The sensor must be in raw or passive mode to access the regular RGB data
   * if the sensor is not in raw or passive mode, it will be placed into raw
   * mode by this method.
   *
   * @return The Raw Green sensor value.
   */
UINT16 HiTechnicColorSensor::GetRawGreen()
{
	UINT16 rawGreen = 0;
	
	if(m_mode == kActive)
	{
		SetMode(kRaw);
	}
	if (m_i2c)
	{
		m_i2c->Read(kRawGreenRegister, sizeof(rawGreen), (UINT8 *)&rawGreen);
	}
	return rawGreen;
}

/**
 * Get the Raw Blue value.
 *
 * Gets the (0-65536) raw blue value from the sensor.
 * 
 * The sensor must be in raw or passive mode to access the regular RGB data
 * if the sensor is not in raw or passive mode, it will be placed into raw
 * mode by this method.
 *
 * @return The Raw Blue sensor value.
 */
UINT16 HiTechnicColorSensor::GetRawBlue()
{
	UINT16 rawBlue = 0;
	
	if(m_mode == kActive)
	{
		SetMode(kRaw);
	}
	if (m_i2c)
	{
		m_i2c->Read(kRawBlueRegister, sizeof(rawBlue), (UINT8 *)&rawBlue);
	}
	return rawBlue;
}

/**
 * Get the raw value of all three colors from a single sensor reading.
 * Using this method ensures that all three values come from the
 * same sensor reading, using the individual color methods provides
 * no such guarantee.
 *
 * Gets the (0-65536) raw color values from the sensor.
 * 
 * The sensor must be in raw or passive mode to access the regular RGB data
 * if the sensor is not in raw or passive mode, it will be placed into raw
 * mode by this method.
 *
 * @return An RGB object with the raw sensor values.
 */
HiTechnicColorSensor::RGB HiTechnicColorSensor::GetRawRGB()
{
	UINT8 colors[6] = {0,0,0,0,0,0};
	RGB result;
	
	if(m_mode != kActive)
	{
		SetMode(kActive);
	}
	if(m_i2c)
	{
		m_i2c->Read(kRedRegister, sizeof(colors), (UINT8*)&colors);
	}
	
	result.red = (colors[0]<<8) + colors[1];
	result.green = (colors[2]<<8) + colors[3];
	result.blue = (colors[4]<<8) + colors[5];
	
	return result;
}

/**
 * Set the Mode of the color sensor
 * This method is used to set the color sensor to one of the three modes,
 * active, passive or raw. The sensor defaults to active mode which uses the
 * internal LED and returns an interpreted color value and 3 8-bit RGB channel
 * values. Raw mode uses the internal LED and returns 3 16-bit RGB channel values.
 * Passive mode disables the internal LED and returns 3 16-bit RGB channel values.
 * @param mode The mode to set
 */
void HiTechnicColorSensor::SetMode(tColorMode mode)
{
	if(m_i2c)
	{
		m_i2c->Write(kModeRegister, (UINT8)mode);
	}
}

/*
 * Live Window code, only does anything if live window is activated.
 */
std::string HiTechnicColorSensor::GetType()
{
    return "Compass";
}

/**
 * {@inheritDoc}
 */
void HiTechnicColorSensor::InitTable(ITable *subtable) {
    m_table = subtable;
    UpdateTable();
}

/**
 * {@inheritDoc}
 */
void HiTechnicColorSensor::UpdateTable() {
    if (m_table != NULL) {
        m_table->PutNumber("Value", GetColor());
    }
}

/**
 * {@inheritDoc}
 */
ITable* HiTechnicColorSensor::GetTable()
{
    return m_table;
}

/**
 * {@inheritDoc}
 */
void HiTechnicColorSensor::StartLiveWindowMode()
{
	
}

/**
 * {@inheritDoc}
 */
void HiTechnicColorSensor::StopLiveWindowMode()
{
	
}
