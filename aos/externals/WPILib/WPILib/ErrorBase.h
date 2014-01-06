/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef _ERROR_BASE_H
#define _ERROR_BASE_H

#include "Base.h"
#include "ChipObject/NiRio.h"
#include "Error.h"

// Helper macros to fill in the context information for you. See the
// documentation for the methods that they call for details.
#define wpi_setErrnoErrorWithContext(context) \
    (this->SetErrnoError((context), __FILE__, __FUNCTION__, __LINE__))
#define wpi_setErrnoError() \
    (wpi_setErrnoErrorWithContext(""))
#define wpi_setImaqErrorWithContext(code, context) \
    (this->SetImaqError((code), (context), __FILE__, __FUNCTION__, __LINE__))
#define wpi_setErrorWithContext(code, context) \
        (this->SetError((code), (context), __FILE__, __FUNCTION__, __LINE__))
#define wpi_setError(code)   (wpi_setErrorWithContext(code, ""))
#define wpi_setStaticErrorWithContext(object, code, context) \
        (object->SetError((code), (context), __FILE__, __FUNCTION__, __LINE__))
#define wpi_setStaticError(object, code) \
    (wpi_setStaticErrorWithContext(object, code, ""))
#define wpi_setGlobalErrorWithContext(code, context) \
        (ErrorBase::SetGlobalError((code), (context), \
                                   __FILE__, __FUNCTION__, __LINE__))
#define wpi_setGlobalError(code)   (wpi_setGlobalErrorWithContext(code, ""))
#define wpi_setWPIErrorWithContext(error, context) \
        (this->SetWPIError((wpi_error_s_##error), (context), \
                           __FILE__, __FUNCTION__, __LINE__))
#define wpi_setWPIError(error)   (wpi_setWPIErrorWithContext(error, ""))
#define wpi_setStaticWPIErrorWithContext(object, error, context) \
        (object->SetWPIError((wpi_error_s_##error), (context), \
                             __FILE__, __FUNCTION__, __LINE__))
#define wpi_setStaticWPIError(object, error) \
    (wpi_setStaticWPIErrorWithContext(object, error, ""))
#define wpi_setGlobalWPIErrorWithContext(error, context) \
        (ErrorBase::SetGlobalWPIError((wpi_error_s_##error), (context), \
                                      __FILE__, __FUNCTION__, __LINE__))
#define wpi_setGlobalWPIError(error) \
    (wpi_setGlobalWPIErrorWithContext(error, ""))

/**
 * Base class for most objects.
 * ErrorBase is the base class for most objects since it holds the generated error
 * for that object. In addition, there is a single instance of a global error object
 *
 * BE AWARE: This does include a mutable instance variable! This means that even
 * if you make an object const it's not really. However, all modification to
 * that instance variable is protected by a semaphore, so it does not create any
 * thread safety issues.
 *
 * All of the Set*Error methods will update the global error if there is nothing
 * there already.
 */
class ErrorBase
{
public:
	ErrorBase();
	virtual ~ErrorBase();

	Error& GetError() const;
	void SetErrnoError(const char *contextMessage,
		const char* filename, const char* function, uint32_t lineNumber) const;
	void SetImaqError(int success, const char *contextMessage,
        const char* filename, const char* function, uint32_t lineNumber) const;
	void SetError(Error::Code code, const char *contextMessage,
		const char* filename, const char* function, uint32_t lineNumber) const;
	void SetWPIError(const char *errorMessage, const char *contextMessage,
		const char* filename, const char* function, uint32_t lineNumber) const;
	void CloneError(const ErrorBase *rhs) const;
	void ClearError() const;
	bool StatusIsFatal() const;
	static void SetGlobalError(Error::Code code, const char *contextMessage,
		const char* filename, const char* function, uint32_t lineNumber);
	static void SetGlobalWPIError(const char *errorMessage, const char *contextMessage,
		const char* filename, const char* function, uint32_t lineNumber);
	static const Error& GetGlobalError();

protected:
  // This mutable is safe because Error guarantees that all modifications are
  // protected with an internal lock.
	mutable Error m_error;
	// TODO: Replace globalError with a global list of all errors, but make sure
  // that it's thread safe.
	static Error _globalError;

private:
	DISALLOW_COPY_AND_ASSIGN(ErrorBase);
};

#endif
