/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "Commands/PIDSubsystem.h"
#include "SmartDashboard/SendablePIDController.h"
#include "float.h"

// XXX max and min are not used?

PIDSubsystem::PIDSubsystem(const char *name, double p, double i, double d) :
	Subsystem(name)
{
	m_max = DBL_MAX;
	m_min = DBL_MIN;
	m_controller = new SendablePIDController(p, i, d, this, this);
}

PIDSubsystem::PIDSubsystem(const char *name, double p, double i, double d,
	double period) :
	Subsystem(name)
{
	m_max = DBL_MAX;
	m_min = DBL_MIN;
	m_controller = new SendablePIDController(p, i, d, this, this, period);
}

PIDSubsystem::PIDSubsystem(double p, double i, double d) :
	Subsystem("PIDSubsystem")
{
	m_max = DBL_MAX;
	m_min = DBL_MIN;
	m_controller = new SendablePIDController(p, i, d, this, this);
}

PIDSubsystem::PIDSubsystem(double p, double i, double d, double period) :
	Subsystem("PIDSubsystem")
{
	m_max = DBL_MAX;
	m_min = DBL_MIN;
	m_controller = new SendablePIDController(p, i, d, this, this, period);
}

PIDSubsystem::~PIDSubsystem()
{
	delete m_controller;
}

void PIDSubsystem::Enable()
{
	m_controller->Enable();
}

void PIDSubsystem::Disable()
{
	m_controller->Disable();
}

std::string PIDSubsystem::GetType()
{
	return "PIDSubsystem";
}

NetworkTable *PIDSubsystem::GetControllerTable()
{
	return m_controller->GetTable();
}

PIDController *PIDSubsystem::GetPIDController()
{
	return m_controller;
}

void PIDSubsystem::SetSetpoint(double setpoint)
{
	m_controller->SetSetpoint(setpoint);
}

void PIDSubsystem::SetSetpointRelative(double deltaSetpoint)
{
	SetSetpoint(GetSetpoint() + deltaSetpoint);
}

double PIDSubsystem::GetSetpoint()
{
	return m_controller->GetSetpoint();
}

double PIDSubsystem::GetPosition()
{
	return ReturnPIDInput();
}

void PIDSubsystem::SetSetpointRange(double a, double b)
{
	if (a <= b)
	{
		m_min = a;
		m_max = b;
	}
	else
	{
		m_min = b;
		m_max = a;
	}
}

void PIDSubsystem::PIDWrite(float output)
{
	UsePIDOutput(output);
}

double PIDSubsystem::PIDGet()
{
	return ReturnPIDInput();
}
