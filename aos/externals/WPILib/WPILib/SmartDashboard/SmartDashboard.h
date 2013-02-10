/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __SMART_DASHBOARD_H__
#define __SMART_DASHBOARD_H__

#include "SensorBase.h"
#include <map>
#include <string>

class NetworkTable;
class SmartDashboardData;
class SmartDashboardNamedData;

class SmartDashboard : public SensorBase
{
public:
	static SmartDashboard *GetInstance();

	void PutData(const char *keyName, SmartDashboardData *value);
	void PutData(SmartDashboardNamedData *value);
	SmartDashboardData* GetData(const char *keyName);
	void PutBoolean(const char *keyName, bool value);
	bool GetBoolean(const char *keyName);
	void PutInt(const char *keyName, int value);
	int GetInt(const char *keyName);
	void PutDouble(const char *keyName, double value);
	double GetDouble(const char *keyName);
	void PutString(const char *keyName, const char *value);
	int GetString(const char *keyName, char *value, int valueLen);
	std::string GetString(std::string keyName);
	void PutString(std::string keyName, std::string value);

	void init();
	static int LogChar(char value, const char *name);
	static int LogChar(wchar_t value, const char *name);
	static int Log(INT32 value, const char *name);
	static int Log(INT64 value, const char *name);
	static int Log(bool value, const char *name);
	static int Log(float value, const char *name);
	static int Log(double value, const char *name);
	static int Log(const char *value, const char *name);

private:
	SmartDashboard();
	virtual ~SmartDashboard();
	DISALLOW_COPY_AND_ASSIGN(SmartDashboard);

	static SmartDashboard *_instance;

	/** The {@link NetworkTable} used by {@link SmartDashboard} */
	NetworkTable *m_table;
	/** 
	 * A map linking tables in the SmartDashboard to the {@link SmartDashboardData} objects
	 * they came from.
	 */
	std::map<NetworkTable *, SmartDashboardData *> m_tablesToData;
};

#endif

