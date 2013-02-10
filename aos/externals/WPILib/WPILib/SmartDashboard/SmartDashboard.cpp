/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "SmartDashboard/SmartDashboard.h"

#include "NetworkCommunication/UsageReporting.h"
#include "NetworkTables/NetworkTable.h"
#include "SmartDashboard/SmartDashboardData.h"
#include "SmartDashboard/SmartDashboardNamedData.h"
#include "WPIErrors.h"

SmartDashboard *SmartDashboard::_instance = NULL;

/**
 * The {@link SmartDashboard} class is the bridge between robot programs and the SmartDashboard on the
 * laptop.
 *
 * <p>When a value is put into the SmartDashboard here, it pops up on the SmartDashboard on the laptop.
 * Users can put values into and get values from the SmartDashboard</p>
 */
SmartDashboard::SmartDashboard()
{
	AddToSingletonList();
	m_table = NetworkTable::GetTable("SmartDashboard");

	nUsageReporting::report(nUsageReporting::kResourceType_SmartDashboard, nUsageReporting::kSmartDashboard_Instance);
}

SmartDashboard::~SmartDashboard()
{
}

/**
 * Get the one and only {@link SmartDashboard} object
 * @return pointer to the {@link SmartDashboard}
 */
SmartDashboard *SmartDashboard::GetInstance()
{
	if (_instance == NULL)
	{
		_instance = new SmartDashboard();
	}
	return _instance;
}


/**
 * Maps the specified key to the specified value in this table.
 * The key can not be NULL.
 * The value can be retrieved by calling the get method with a key that is equal to the original key.
 * @param keyName the key
 * @param value the value
 */
void SmartDashboard::PutData(const char *keyName, SmartDashboardData *value)
{
	if (keyName == NULL)
	{
		wpi_setWPIErrorWithContext(NullParameter, "keyName");
		return;
	}
	if (value == NULL)
	{
		wpi_setWPIErrorWithContext(NullParameter, "value");
		return;
	}
	NetworkTable *type = new NetworkTable();
	type->PutString("~TYPE~", value->GetType());
	type->PutSubTable("Data", value->GetTable());
	m_table->PutSubTable(keyName, type);
	m_tablesToData[type] = value;
}

/**
 * Maps the specified key (where the key is the name of the {@link SmartDashboardNamedData}
 * to the specified value in this table.
 * The value can be retrieved by calling the get method with a key that is equal to the original key.
 * @param value the value
 */
void SmartDashboard::PutData(SmartDashboardNamedData *value)
{
	if (value == NULL)
	{
		wpi_setWPIErrorWithContext(NullParameter, "value");
		return;
	}
	PutData(value->GetName().c_str(), value);
}

/**
 * Returns the value at the specified key.
 * @param keyName the key
 * @return the value
 */
SmartDashboardData *SmartDashboard::GetData(const char *keyName)
{
	if (keyName == NULL)
	{
		wpi_setWPIErrorWithContext(NullParameter, "keyName");
		return NULL;
	}
	NetworkTable *subtable = m_table->GetSubTable(keyName);
	SmartDashboardData *data = m_tablesToData[subtable];
	if (data == NULL)
	{
		wpi_setWPIErrorWithContext(SmartDashboardMissingKey, keyName);
		return NULL;
	}
    return data;
}

/**
 * Maps the specified key to the specified value in this table.
 * The key can not be NULL.
 * The value can be retrieved by calling the get method with a key that is equal to the original key.
 * @param keyName the key
 * @param value the value
 */
void SmartDashboard::PutBoolean(const char *keyName, bool value)
{
	m_table->PutBoolean(keyName, value);
}

/**
 * Returns the value at the specified key.
 * @param keyName the key
 * @return the value
 */
bool SmartDashboard::GetBoolean(const char *keyName)
{
	return m_table->GetBoolean(keyName);
}

/**
* Maps the specified key to the specified value in this table.
* The keyName can not be NULL.
* The value can be retrieved by calling the get method with a key that is equal to the original key.
* @param keyName the key
* @param value the value
*/
void SmartDashboard::PutInt(const char *keyName, int value)
{
	m_table->PutInt(keyName, value);
}

/**
 * Returns the value at the specified key.
 * @param keyName the key
 * @return the value
 */
int SmartDashboard::GetInt(const char *keyName)
{
	return m_table->GetInt(keyName);
}

/**
 * Maps the specified key to the specified value in this table.
 * The key can not be NULL.
 * The value can be retrieved by calling the get method with a key that is equal to the original key.
 * @param keyName the key
 * @param value the value
 */
void SmartDashboard::PutDouble(const char *keyName, double value)
{
	m_table->PutDouble(keyName, value);
}

/**
 * Returns the value at the specified key.
 * @param keyName the key
 * @return the value
 */
double SmartDashboard::GetDouble(const char *keyName)
{
	return m_table->GetDouble(keyName);
}

/**
 * Maps the specified key to the specified value in this table.
 * Neither the key nor the value can be NULL.
 * The value can be retrieved by calling the get method with a key that is equal to the original key.
 * @param keyName the key
 * @param value the value
 */
void SmartDashboard::PutString(const char *keyName, const char *value)
{
	m_table->PutString(keyName, value);
}

/**
 * Returns the value at the specified key.
 * @param keyName the key
 * @param value the buffer to fill with the value
 * @param valueLen the size of the buffer pointed to by value
 * @return the length of the string
 */
int SmartDashboard::GetString(const char *keyName, char *value, int valueLen)
{
	return m_table->GetString(keyName, value, valueLen);
}

/**
 * Maps the specified key to the specified value in this table.
 * Neither the key nor the value can be NULL.
 * The value can be retrieved by calling the get method with a key that is equal to the original key.
 * @param keyName the key
 * @param value the value
 */
void SmartDashboard::PutString(std::string keyName, std::string value)
{
	m_table->PutString(keyName, value);
}

/**
 * Returns the value at the specified key.
 * @param keyName the key
 * @return the value
 */
std::string SmartDashboard::GetString(std::string keyName)
{
	return m_table->GetString(keyName);
}

/**
 * @deprecated no longer necessary
 */
void SmartDashboard::init()
{
}

/**
 * Send the given byte value to the client as the field with the given name.
 * @param value The value to be displayed on the client.
 * @param name The name of the field.
 * @return An integer status code.
 * @deprecated use {@link SmartDashboard#PutInt(const char*, int)}
 */
int SmartDashboard::LogChar(char value, const char *name)
{
	GetInstance()->PutInt(name, value);
	return 0;
}

/**
 * Send the given UTF-16 char value to the client as the field with the given name.
 * @param value The value to be displayed on the client.
 * @param name The name of the field.
 * @return An integer status code.
 * @deprecated use {@link SmartDashboard#PutInt(const char*, int)}
 */
int SmartDashboard::LogChar(wchar_t value, const char *name)
{
	GetInstance()->PutInt(name, value);
	return 0;
}

/**
 * Sends the given int value to the client as the field with the given name.
 * @param value The value to send.
 * @param name The name of the field.
 * @return An integer status code.
 * @deprecated use {@link SmartDashboard#PutInt(const char*, int)}
 */
int SmartDashboard::Log(INT32 value, const char *name)
{
	GetInstance()->PutInt(name, value);
	return 0;
}

/**
 * Sends the given long value to the client as the field with the given name.
 * @param value The value to send.
 * @param name The name of the field.
 * @return An integer status code.
 * @deprecated use {@link SmartDashboard#PutDouble(const char*, double)}
 */
int SmartDashboard::Log(INT64 value, const char *name)
{
	GetInstance()->PutDouble(name, value);
	return 0;
}

/**
 * Sends the given boolean value to the client as the field with the given name.
 * @param value The value to send.
 * @param name The name of the field.
 * @return An integer status code.
 * @deprecated use {@link SmartDashboard#PutBoolean(const char*, bool)} instead
 */
int SmartDashboard::Log(bool value, const char *name)
{
	GetInstance()->PutBoolean(name, value);
	return 0;
}

/**
 * Sends the given float value to the client as the field with the given name.
 * @param value The value to send.
 * @param name The name of the field.
 * @return An integer status code.
 * @deprecated use {@link SmartDashboard#PutDouble(const char*, double)}
 */
int SmartDashboard::Log(float value, const char *name)
{
	GetInstance()->PutDouble(name, value);
	return 0;
}

/**
 * Sends the given double value to the client as the field with the given name.
 * @param value The value to send.
 * @param name The name of the field.
 * @return An integer status code.
 * @deprecated use {@link SmartDashboard#PutDouble(const char*, double)}
 */
int SmartDashboard::Log(double value, const char *name)
{
	GetInstance()->PutDouble(name, value);
	return 0;
}

/**
 * Sends the given string value to the client as the field with the given name.
 * @param value The value to send. This may be at most 63 characters in length.
 * @param name The name of the field.
 * @return An integer status code.
 * @deprecated use {@link SmartDashboard#PutString(const char*, const char*)}
 */
int SmartDashboard::Log(const char* value, const char *name)
{
	GetInstance()->PutString(name, value);
	return 0;
}
