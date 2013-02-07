/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __DASHBOARD_H__
#define __DASHBOARD_H__

#include "DashboardBase.h"
#include "NetworkCommunication/FRCComm.h"
#include <stack>
#include <vector>
#include <vxWorks.h>

/**
 * Pack data into the "user data" field that gets sent to the dashboard laptop
 * via the driver station.
 */
class Dashboard : public DashboardBase
{
public:
	explicit Dashboard(SEM_ID statusDataSemaphore);
	virtual ~Dashboard();

	enum Type {kI8, kI16, kI32, kU8, kU16, kU32, kFloat, kDouble, kBoolean, kString, kOther};
	enum ComplexType {kArray, kCluster};

	void AddI8(INT8 value);
	void AddI16(INT16 value);
	void AddI32(INT32 value);
	void AddU8(UINT8 value);
	void AddU16(UINT16 value);
	void AddU32(UINT32 value);
	void AddFloat(float value);
	void AddDouble(double value);
	void AddBoolean(bool value);
	void AddString(char* value);
	void AddString(char* value, INT32 length);

	void AddArray();
	void FinalizeArray();
	void AddCluster();
	void FinalizeCluster();

	void Printf(const char *writeFmt, ...);

	INT32 Finalize();
	void GetStatusBuffer(char** userStatusData, INT32* userStatusDataSize);
	void Flush() {}
private:
	static const INT32 kMaxDashboardDataSize = USER_STATUS_DATA_SIZE - sizeof(UINT32) * 3 - sizeof(UINT8); // 13 bytes needed for 3 size parameters and the sequence number

	// Usage Guidelines...
	DISALLOW_COPY_AND_ASSIGN(Dashboard);

	bool ValidateAdd(INT32 size);
	void AddedElement(Type type);
	bool IsArrayRoot();

	char *m_userStatusData;
	INT32 m_userStatusDataSize;
	char *m_localBuffer;
	char *m_localPrintBuffer;
	char *m_packPtr;
	std::vector<Type> m_expectedArrayElementType;
	std::vector<INT32> m_arrayElementCount;
	std::vector<INT32*> m_arraySizePtr;
	std::stack<ComplexType> m_complexTypeStack;
	SEM_ID m_printSemaphore;
	SEM_ID m_statusDataSemaphore;
};

#endif

