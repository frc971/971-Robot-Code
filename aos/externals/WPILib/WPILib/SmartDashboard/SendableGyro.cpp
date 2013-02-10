/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "SmartDashboard/SendableGyro.h"

#include "NetworkTables/NetworkTable.h"
#include "WPIErrors.h"

#include <taskLib.h>

/** The time (in seconds) between updates to the table */
static const double kDefaultTimeBetweenUpdates = 0.2;
static const char *kAngle = "angle";

/**
 * Gyro constructor given a moduleNumber and a channel.
 *
 * @param moduleNumber The analog module the gyro is connected to.
 * @param channel The analog channel the gyro is connected to.
 */
SendableGyro::SendableGyro(UINT8 moduleNumber, UINT32 channel) :
	Gyro(moduleNumber, channel),
	m_offset(0.0),
	m_period(kDefaultTimeBetweenUpdates),
	m_table(NULL),
	m_publisher("SendableGyroPublisher", (FUNCPTR)InitPublishTask),
	m_runPublisher(false)
{
}

/**
 * Gyro constructor with only a channel.
 *
 * Use the default analog module slot.
 *
 * @param channel The analog channel the gyro is connected to.
 */
SendableGyro::SendableGyro(UINT32 channel) :
	Gyro(channel),
	m_offset(0.0),
	m_period(kDefaultTimeBetweenUpdates),
	m_table(NULL),
	m_publisher("SendableGyroPublisher", (FUNCPTR)InitPublishTask),
	m_runPublisher(false)
{
}

/**
 * Gyro constructor with a precreated analog channel object.
 * Use this constructor when the analog channel needs to be shared. There
 * is no reference counting when an AnalogChannel is passed to the gyro.
 * @param channel The AnalogChannel object that the gyro is connected to.
 */
SendableGyro::SendableGyro(AnalogChannel* channel):  
	Gyro(channel),
	m_offset(0.0),
	m_period(kDefaultTimeBetweenUpdates),
	m_table(NULL),
	m_publisher("SendableGyroPublisher", (FUNCPTR)InitPublishTask),
	m_runPublisher(false)
{
}

SendableGyro::~SendableGyro()
{
	if (m_table != NULL)
	{
		// Stop the task
		m_runPublisher = false;
		while(m_publisher.Verify())
			taskDelay(10);

		// Stop listening to the table
		m_table->RemoveChangeListener(kAngle, this);

		delete m_table;
		m_table = NULL;
	}
}

float SendableGyro::GetAngle()
{
	return m_offset + Gyro::GetAngle();
}

void SendableGyro::Reset()
{
	m_offset = 0.0;
	Gyro::Reset();
}

/**
 * Sets the time (in seconds) between updates to the {@link SmartDashboard}.
 * The default is 0.2 seconds.
 * @param period the new time between updates
 */
void SendableGyro::SetUpdatePeriod(double period)
{
	if (period <= 0.0)
		wpi_setWPIErrorWithContext(ParameterOutOfRange, "period <= 0.0");
	else
		m_period = period;
}

/**
 * Returns the period (in seconds) between updates to the {@link SmartDashboard}.
 * This value is independent of whether or not this {@link SendableGyro} is connected
 * to the {@link SmartDashboard}.  The default value is 0.2 seconds.
 * @return the period (in seconds)
 */
double SendableGyro::GetUpdatePeriod()
{
	return m_period;
}

/**
 * Reset the gyro.
 * Resets the gyro to the given heading. This can be used if there is significant
 * drift in the gyro and it needs to be recalibrated after it has been running.
 * @param angle the angle the gyro should believe it is pointing
 */
void SendableGyro::ResetToAngle(double angle)
{
	m_offset = angle;
	Gyro::Reset();
}

NetworkTable *SendableGyro::GetTable()
{
	if (m_table == NULL)
	{
		m_table = new NetworkTable();
		m_table->PutInt(kAngle, (int)GetAngle());
		m_table->AddChangeListener(kAngle, this);
		m_publisher.Start((UINT32)this);
	}
	return m_table;
}

void SendableGyro::ValueChanged(NetworkTable *table, const char *name, NetworkTables_Types type)
{
	// Update value from smart dashboard
	ResetToAngle(m_table->GetDouble(name));
}

void SendableGyro::PublishTaskRun()
{
	m_runPublisher = true;
	while(m_runPublisher)
	{
		m_table->PutInt(kAngle, (int)GetAngle());
		taskDelay((INT32)(m_period * 1000));
	}
}
