/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __PID_COMMAND_H__
#define __PID_COMMAND_H__

#include "Commands/Command.h"
#include "PIDSource.h"
#include "PIDOutput.h"

class NetworkTable;
class PIDController;
class SendablePIDController;

class PIDCommand : public Command, public PIDOutput, public PIDSource
{
public:
	PIDCommand(const char *name, double p, double i, double d);
	PIDCommand(const char *name, double p, double i, double d, double period);
	PIDCommand(double p, double i, double d);
	PIDCommand(double p, double i, double d, double period);
	virtual ~PIDCommand();
	
	void SetSetpointRelative(double deltaSetpoint);

	// PIDOutput interface
	virtual void PIDWrite(float output);

	// PIDSource interface
	virtual double PIDGet();

	NetworkTable *GetControllerTable();

	virtual std::string GetType();
protected:
	PIDController *GetPIDController();
	virtual void _Initialize();
	virtual void _Interrupted();
	virtual void _End();
	void SetSetpoint(double setpoint);
	double GetSetpoint();
	double GetPosition();
	void SetSetpointRange(double a, double b);

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

