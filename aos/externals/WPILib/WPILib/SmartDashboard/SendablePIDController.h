/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __SENDABLE_PID_CONTROLLER_H__
#define __SENDABLE_PID_CONTROLLER_H__

#include "NetworkTables/NetworkTableChangeListener.h"
#include "PIDController.h"
#include "SmartDashboard/SmartDashboardData.h"

class NetworkTable;

/**
 * A {@link SendablePIDController} is a {@link PIDController} that can be sent over to the {@link SmartDashboard} using
 * the {@link SmartDashboard#PutData(const char *, SmartDashboardData *) PutData(...)}
 * method.
 *
 * <p>It is useful if you want to test values of a {@link PIDController} without having to recompile code between tests.
 * Also, consider using {@link Preferences} to save the important values between matches.</p>
 * 
 * @see SmartDashboard
 */
class SendablePIDController : public PIDController, public SmartDashboardData, public NetworkTableChangeListener
{
public:
	SendablePIDController(double p, double i, double d, PIDSource *source, PIDOutput *output);
	SendablePIDController(double p, double i, double d, PIDSource *source, PIDOutput *output, double period);
	virtual ~SendablePIDController();

	virtual void SetSetpoint(float setpoint);
	virtual void SetPID(double p, double i, double d); 
	virtual void Enable();
	virtual void Disable();

	// SmartDashboardData interface
	virtual std::string GetType() {return "PIDController";}
	virtual NetworkTable *GetTable();

	// NetworkTableChangeListener interface
	virtual void ValueChanged(NetworkTable *table, const char *name, NetworkTables_Types type);
	virtual void ValueConfirmed(NetworkTable *table, const char *name, NetworkTables_Types type) {}

private:
	NetworkTable* m_table;
};

#endif
