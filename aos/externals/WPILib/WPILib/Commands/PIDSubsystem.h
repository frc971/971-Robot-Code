/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __PID_SUBSYSTEM_H__
#define __PID_SUBSYSTEM_H__

#include "Commands/Subsystem.h"
#include "PIDController.h"
#include "PIDSource.h"
#include "PIDOutput.h"

class NetworkTable;
class SendablePIDController;

class PIDSubsystem : public Subsystem, public PIDOutput, public PIDSource
{
public:
	PIDSubsystem(const char *name, double p, double i, double d);
	PIDSubsystem(const char *name, double p, double i, double d, double period);
	PIDSubsystem(double p, double i, double d);
	PIDSubsystem(double p, double i, double d, double period);
	virtual ~PIDSubsystem();
	
	void Enable();
	void Disable();
	NetworkTable *GetControllerTable();

	// SmartDashboardData interface
	virtual std::string GetType();

	// PIDOutput interface
	virtual void PIDWrite(float output);

	// PIDSource interface
	virtual double PIDGet();
	void SetSetpoint(double setpoint);
	void SetSetpointRelative(double deltaSetpoint);
	double GetSetpoint();
	double GetPosition();
	void SetSetpointRange(double a, double b);

protected:
	PIDController *GetPIDController();

	virtual double ReturnPIDInput() = 0;
	virtual void UsePIDOutput(double output) = 0;

private:
	/** The max setpoint value */
	double m_max;
	/** The min setpoint value */
	double m_min;
	/** The internal {@link PIDController} */
	SendablePIDController *m_controller;
};

#endif

