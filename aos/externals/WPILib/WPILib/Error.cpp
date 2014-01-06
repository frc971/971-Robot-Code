/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "Error.h"

#include <taskLib.h>
#include <cstdio>
#include <cstring>

#include "NetworkCommunication/FRCComm.h"
#include "Timer.h"
#include "Utility.h"
bool Error::m_stackTraceEnabled = false;
bool Error::m_suspendOnErrorEnabled = false;

Error::Error()
	: m_code(0)
	, m_lineNumber(0)
	, m_originatingObject(NULL)
	, m_timestamp (0.0)
{}

Error::~Error()
{}

/**
 * Clones another error into this if this is currently clear. If not, does
 * nothing.
 * This is necessary because just using "if (!IsClear()) Clone(error)" has a
 * race condition which this method does not.
 * Cloning 2 errors into each other at the same time can lead to deadlocks!
 */
void Error::CloneIfClear(const Error &error) {
  Synchronized sync(m_semaphore);
  if (IsClear()) {
    DoClone(error);
  }
}

/**
 * Clones another error into this object.
 * Cloning 2 errors into each other at the same time can lead to deadlocks!
 */
void Error::Clone(const Error &error) {
  Synchronized sync(m_semaphore);
  DoClone(error);
}

void Error::DoClone(const Error &error)
{
  Synchronized sync(error.m_semaphore);
	m_code = error.m_code;
	m_message = error.m_message;
	m_filename = error.m_filename;
	m_function = error.m_function;
	m_lineNumber = error.m_lineNumber;
	m_originatingObject = error.m_originatingObject;
	m_timestamp = error.m_timestamp;
}

bool Error::IsClear() const { return GetCode() == 0; }

Error::Code Error::GetCode() const
{	return m_code;  }

std::string Error::GetMessage() const
{	return m_message;  }

std::string Error::GetFilename() const
{	return m_filename;  }

std::string Error::GetFunction() const
{	return m_function;  }

uint32_t Error::GetLineNumber() const
{	return m_lineNumber;  }

const ErrorBase* Error::GetOriginatingObject() const
{	return m_originatingObject;  }

double Error::GetTime() const
{	return m_timestamp;  }

void Error::Set(Code code, const char* contextMessage, const char* filename,
                const char* function, uint32_t lineNumber,
                const ErrorBase* originatingObject)  {
  Synchronized sync(m_semaphore);
	m_code = code;
	m_message = contextMessage;
	m_filename = filename;
	m_function = function;
	m_lineNumber = lineNumber;
	m_originatingObject = originatingObject;
	m_timestamp = GetTime();

	Report();

	if (m_suspendOnErrorEnabled) taskSuspend(0 /*self*/);
}

void Error::Report() const
{
	// Error string buffers
	char error[256];
	char error_with_code[256];

	// Build error strings
	if (m_code != -1 && m_code != 1)
	{
		snprintf(error, sizeof(error),
             "%s: status = %d (0x%08X) %s ...in %s() in %s at line %d\n",
				     m_code < 0 ? "ERROR" : "WARNING", (int32_t)m_code,
             (uint32_t)m_code, m_message.c_str(),
				     m_function.c_str(), m_filename.c_str(), m_lineNumber);
		snprintf(error_with_code, sizeof(error_with_code),
             "<Code>%ld %s", (int32_t)m_code, error);
	} else {
		snprintf(error, sizeof(error),
             "%s: %s ...in %s() in %s at line %d\n",
             m_code < 0 ? "ERROR" : "WARNING", m_message.c_str(),
				     m_function.c_str(), m_filename.c_str(), m_lineNumber);
		strncpy(error_with_code, error, sizeof(error_with_code));
	}
	// TODO: Add logging to disk

	// Send to the DriverStation
	setErrorData(error_with_code, strlen(error_with_code), 100);

	// Print to console
	printf("\n\n>>>>%s", error);

	if (m_stackTraceEnabled)
	{
		printf("-----------<Stack Trace>----------------\n");
		wpi_selfTrace();
	}
}

void Error::Clear()
{
  Synchronized sync(m_semaphore);
	m_code = 0;
	m_message = "";
	m_filename = "";
	m_function = "";
	m_lineNumber = 0;
	m_originatingObject = NULL;
	m_timestamp = 0.0;
}

