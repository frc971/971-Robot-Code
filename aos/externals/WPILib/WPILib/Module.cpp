/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "Module.h"
#include "AnalogModule.h"
#include "DigitalModule.h"
//#include "SolenoidModule.h"
#include "Utility.h"

ReentrantSemaphore Module::m_semaphore;

Module* Module::m_modules[kMaxModules] = {NULL};

/**
 * Constructor.
 * 
 * @param type The type of module represented.
 * @param number The module index within the module type.
 */
Module::Module(nLoadOut::tModuleType type, uint8_t number)
	: m_moduleType (type)
	, m_moduleNumber (number)
{
  Synchronized sync(m_semaphore);
	m_modules[ToIndex(type, number)] = this;
}

/**
 * Destructor.
 */
Module::~Module()
{
  m_modules[ToIndex(m_moduleType, m_moduleNumber)] = NULL;
}

/**
 * Static module singleton factory.
 * 
 * @param type The type of module represented.
 * @param number The module index within the module type.
 */
Module* Module::GetModule(nLoadOut::tModuleType type, uint8_t number)
{
  Synchronized sync(m_semaphore);
	if (m_modules[ToIndex(type, number)] == NULL)
	{
		switch(type)
		{
		case nLoadOut::kModuleType_Analog:
			new AnalogModule(number);
			break;
		case nLoadOut::kModuleType_Digital:
			new DigitalModule(number);
			break;
/*
		case nLoadOut::kModuleType_Solenoid:
			new SolenoidModule(number);
			break;
*/
		default:
		    return NULL;
		}
	}
	return m_modules[ToIndex(type, number)];
}

/**
 * Create an index into the m_modules array based on type and number
 * 
 * @param type The type of module represented.
 * @param number The module index within the module type.
 * @return The index into m_modules.
 */
uint8_t Module::ToIndex(nLoadOut::tModuleType type, uint8_t number)
{
	if (number == 0 || number > kMaxModuleNumber) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Trying to get index for invalid module %d",
             static_cast<int>(number));
    wpi_assertWithMessage(false, buf);
    return 0;
  }
	if (type < nLoadOut::kModuleType_Analog ||
      type > nLoadOut::kModuleType_Solenoid) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Trying to get index for invalid module type %d",
             static_cast<int>(type));
    wpi_assertWithMessage(false, buf);
    return 0;
  }
	return (type * kMaxModuleNumber) + (number - 1);
}
