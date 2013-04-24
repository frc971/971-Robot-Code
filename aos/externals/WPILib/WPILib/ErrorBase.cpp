/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "ErrorBase.h"
#include "Synchronized.h"
#include "nivision.h"
#define WPI_ERRORS_DEFINE_STRINGS
#include "WPIErrors.h"

#include <errnoLib.h>
#include <symLib.h>
#include <sysSymTbl.h>

Error ErrorBase::_globalError;

/**
 * @brief Initialize the instance status to 0 for now.
 */
ErrorBase::ErrorBase()
{}

ErrorBase::~ErrorBase()
{}

/**
 * @brief Retrieve the current error.
 * Get the current error information associated with this sensor.
 */
Error& ErrorBase::GetError() const
{
	return m_error;
}

/**
 * @brief Clear the current error information associated with this sensor.
 */
void ErrorBase::ClearError() const
{
	m_error.Clear();
}

/**
 * @brief Set error information associated with a C library call that set an
 * error to the "errno" "global variable" (it's really a macro that calls a
 * function so that it's thread safe).
 * 
 * @param contextMessage A custom message from the code that set the error.
 * @param filename Filename of the error source
 * @param function Function of the error source
 * @param lineNumber Line number of the error source
 */
void ErrorBase::SetErrnoError(const char *contextMessage,
		const char* filename, const char* function, UINT32 lineNumber) const
{
	char err[256];
	int errNo = errnoGet();
	if (errNo == 0)
	{
		sprintf(err, "OK: %s", contextMessage);
	}
	else
	{
		char *statName = new char[MAX_SYS_SYM_LEN + 1];
		int pval;
		SYM_TYPE ptype;
		symFindByValue(statSymTbl, errNo, statName, &pval, &ptype);
		if (pval != errNo)
			snprintf(err, sizeof(err), "Unknown errno 0x%08X: %s", errNo, contextMessage);
		else
			snprintf(err, sizeof(err), "%s (0x%08X): %s", statName, errNo, contextMessage);
		delete [] statName;
	}

	//  Set the current error information for this object.
	m_error.Set(-1, err, filename, function, lineNumber, this);

  _globalError.CloneIfClear(m_error);
}

/**
 * @brief Set the current error information associated from the nivision Imaq API.
 * 
 * @param success The return from the function
 * @param contextMessage A custom message from the code that set the error.
 * @param filename Filename of the error source
 * @param function Function of the error source
 * @param lineNumber Line number of the error source
 */
void ErrorBase::SetImaqError(int success, const char *contextMessage, const char* filename, const char* function, UINT32 lineNumber) const
{
	//  If there was an error
	if (success <= 0) {
		char err[256];
		sprintf(err, "%s: %s", contextMessage, imaqGetErrorText(imaqGetLastError()));

		//  Set the current error information for this object.
		m_error.Set(imaqGetLastError(), err, filename, function, lineNumber, this);

    _globalError.CloneIfClear(m_error);
	}
}

/**
 * @brief Set the current error information associated with this sensor.
 * 
 * @param code The error code
 * @param contextMessage A custom message from the code that set the error.
 * @param filename Filename of the error source
 * @param function Function of the error source
 * @param lineNumber Line number of the error source
 */
void ErrorBase::SetError(Error::Code code, const char *contextMessage,
		const char* filename, const char* function, UINT32 lineNumber) const
{
	//  If there was an error
	if (code != 0) {
		//  Set the current error information for this object.
		m_error.Set(code, contextMessage, filename, function, lineNumber, this);

    _globalError.CloneIfClear(m_error);
	}
}

/**
 * @brief Set the current error information associated with this sensor.
 * 
 * @param errorMessage The error message from WPIErrors.h
 * @param contextMessage A custom message from the code that set the error.
 * @param filename Filename of the error source
 * @param function Function of the error source
 * @param lineNumber Line number of the error source
 */
void ErrorBase::SetWPIError(const char *errorMessage, const char *contextMessage,
		const char* filename, const char* function, UINT32 lineNumber) const
{
	char err[256];
	sprintf(err, "%s: %s", errorMessage, contextMessage);

	//  Set the current error information for this object.
	m_error.Set(-1, err, filename, function, lineNumber, this);

  _globalError.CloneIfClear(m_error);
}

void ErrorBase::CloneError(const ErrorBase *rhs) const
{
	m_error.Clone(rhs->GetError());
}

/**
@brief Check if the current error code represents a fatal error.
  
@return true if the current error is fatal.
*/
bool ErrorBase::StatusIsFatal() const
{
	return m_error.GetCode() < 0;
}

void ErrorBase::SetGlobalError(Error::Code code, const char *contextMessage,
		const char* filename, const char* function, UINT32 lineNumber)
{
	//  Set the current error information for this object.
	_globalError.Set(code, contextMessage, filename, function, lineNumber, NULL);
}

void ErrorBase::SetGlobalWPIError(const char *errorMessage, const char *contextMessage,
        const char* filename, const char* function, UINT32 lineNumber)
{
	char err[256];
	sprintf(err, "%s: %s", errorMessage, contextMessage);

	_globalError.Set(-1, err, filename, function, lineNumber, NULL);
}

/**
  * Retrieve the current global error.    
*/
const Error& ErrorBase::GetGlobalError()
{
	return _globalError;
}

