/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "SmartDashboard/SendablePIDController.h"

#include "NetworkTables/NetworkTable.h"

static const char *kP = "p";
static const char *kI = "i";
static const char *kD = "d";
static const char *kSetpoint = "setpoint";
static const char *kEnabled = "enabled";

/**
 * Allocate a PID object with the given constants for P, I, D, using a 50ms period.
 * @param p the proportional coefficient
 * @param i the integral coefficient
 * @param d the derivative coefficient
 * @param source The PIDSource object that is used to get values
 * @param output The PIDOutput object that is set to the output value
 */
SendablePIDController::SendablePIDController(double p, double i, double d,
	PIDSource* source, PIDOutput* output) :
	PIDController(p, i, d, source, output),
	m_table(NULL)
{
}

/**
 * Allocate a PID object with the given constants for P, I, D
 * @param p the proportional coefficient
 * @param i the integral coefficient
 * @param d the derivative coefficient
 * @param source The PIDSource object that is used to get values
 * @param output The PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations in seconds. This particularly effects calculations of the
 * integral and differential terms. The default is 50ms.
 */
SendablePIDController::SendablePIDController(double p, double i, double d,
	PIDSource* source, PIDOutput* output, double period) :
	PIDController(p, i, d, source, output, period),
	m_table(NULL)
{
}

SendablePIDController::~SendablePIDController()
{
	if (m_table != NULL)
		m_table->RemoveChangeListenerAny(this);
}

/**
 * Set the setpoint for the PIDController
 * @param setpoint the desired setpoint
 */
void SendablePIDController::SetSetpoint(float setpoint)
{
	PIDController::SetSetpoint(setpoint);

	if (m_table != NULL)
	{
		m_table->PutDouble(kSetpoint, setpoint);
	}
}

/**
 * Set the PID Controller gain parameters.
 * Set the proportional, integral, and differential coefficients.
 * @param p Proportional coefficient
 * @param i Integral coefficient
 * @param d Differential coefficient
 */
void SendablePIDController::SetPID(double p, double i, double d)
{
	PIDController::SetPID(p, i, d);

	if (m_table != NULL)
	{
		m_table->PutDouble(kP, p);
		m_table->PutDouble(kI, i);
		m_table->PutDouble(kD, d);
	}
}

/**
 * Begin running the PIDController
 */
void SendablePIDController::Enable()
{
	PIDController::Enable();

	if (m_table != NULL)
		m_table->PutBoolean(kEnabled, true);
}

/**
 * Stop running the PIDController, this sets the output to zero before stopping.
 */
void SendablePIDController::Disable()
{
	PIDController::Disable();

	if (m_table != NULL)
		m_table->PutBoolean(kEnabled, false);
}

NetworkTable* SendablePIDController::GetTable()
{
	if (m_table == NULL)
	{
		m_table = new NetworkTable();

		m_table->PutDouble(kP, GetP());
		m_table->PutDouble(kI, GetI());
		m_table->PutDouble(kD, GetD());
		m_table->PutDouble(kSetpoint, GetSetpoint());
		m_table->PutBoolean(kEnabled, IsEnabled());

		m_table->AddChangeListenerAny(this);
	}
	return m_table;
}

void SendablePIDController::ValueChanged(NetworkTable *table, const char *name, NetworkTables_Types type)
{
	if (strcmp(name, kP) == 0 || strcmp(name, kI) == 0 || strcmp(name, kD) == 0)
	{
		PIDController::SetPID(table->GetDouble(kP), table->GetDouble(kI),
			table->GetDouble(kD));
	}
	else if (strcmp(name, kSetpoint) == 0)
	{
		PIDController::SetSetpoint(table->GetDouble(kSetpoint));
	}
	else if (strcmp(name, kEnabled) == 0)
	{
		if (table->GetBoolean(kEnabled))
			PIDController::Enable();
		else
			PIDController::Disable();
	}
}
