/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __SENDABLE_GYRO_H__
#define __SENDABLE_GYRO_H__

#include "Gyro.h"
#include "NetworkTables/NetworkTableChangeListener.h"
#include "SmartDashboard/SmartDashboardData.h"
#include "Task.h"

class NetworkTable;

/**
 * The {@link SendableGyro} class behaves exactly the same as a {@link Gyro} except that it
 * also implements {@link SmartDashboardData} so that it can be sent over to the {@link SmartDashboard}.
 */
class SendableGyro : public Gyro, public SmartDashboardData, public NetworkTableChangeListener
{
public:
	SendableGyro(UINT8 moduleNumber, UINT32 channel);
	SendableGyro(UINT32 channel);
	SendableGyro(AnalogChannel* channel);
	virtual ~SendableGyro();

	// Gyro overrides
	virtual float GetAngle();
	virtual void Reset();

	void SetUpdatePeriod(double period);
	double GetUpdatePeriod();
	void ResetToAngle(double angle);

	// SmartDashboardData interface
	virtual std::string GetType() {return "Gyro";}
	virtual NetworkTable *GetTable();

private:
	// NetworkTableChangeListener interface
	virtual void ValueChanged(NetworkTable *table, const char *name, NetworkTables_Types type);
	virtual void ValueConfirmed(NetworkTable *table, const char *name, NetworkTables_Types type) {}

	void PublishTaskRun();

	static int InitPublishTask(SendableGyro *obj) {obj->PublishTaskRun();return 0;}

	/** The angle added to the gyro's value */
	double m_offset;
	/** The period (in seconds) between value updates */
	double m_period;
	NetworkTable *m_table;
	Task m_publisher;
	bool m_runPublisher;
};

#endif
