/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef _ERROR_H
#define _ERROR_H

#include "Base.h"
#include "ChipObject/NiRio.h"
#include <string>
#include "Synchronized.h"

//  Forward declarations
class ErrorBase;

/**
 * Represents an error or warning.
 *
 * All methods that can change instance data are protected by a lock so
 * that it is safe to call any methods from multiple tasks at the same time.
 */
class Error
{
public:
  // -1 is other error, 1 is other warning.
	typedef tRioStatusCode Code;

	Error();
	~Error();

	void CloneIfClear(const Error &error);
  void Clone(const Error &error);
	void Clear();
	void Set(Code code, const char* contextMessage, const char* filename,
		const char *function, UINT32 lineNumber, const ErrorBase* originatingObject);

  bool IsClear() const;
	Code GetCode() const;
  // Have to return by value to avoid race conditions using the result for all
  // of these methods.
  std::string GetMessage() const;
  std::string GetFilename() const;
  std::string GetFunction() const;
	UINT32 GetLineNumber() const;
	const ErrorBase* GetOriginatingObject() const;
	double GetTime() const;

  // Enable or disable printing out a stack trace on the console whenever there
  // is an error.
	static void EnableStackTrace(bool enable) { m_stackTraceEnabled=enable; }
  // Enable or disable having any task that gets an error suspend itself.
	static void EnableSuspendOnError(bool enable) { m_suspendOnErrorEnabled=enable; }

private:
  // Deals with notifying other code of this error.
	void Report() const;
  // Actually implements cloning.
  // Does not lock m_semaphore, so callers must.
  void DoClone(const Error &error);

	Code m_code;
	std::string m_message;
	std::string m_filename;
	std::string m_function;
	UINT32 m_lineNumber;
	const ErrorBase* m_originatingObject;
	double m_timestamp;
  // Used for protecting all modifications to instance data.
  // This means that all non-const methods should lock this for (at least most)
  // of their implementations!
  ReentrantSemaphore m_semaphore;

	static bool m_stackTraceEnabled;
	static bool m_suspendOnErrorEnabled;
	DISALLOW_COPY_AND_ASSIGN(Error);
};

#endif

