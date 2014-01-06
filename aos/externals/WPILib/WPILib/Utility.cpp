/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "Utility.h"

#include "NetworkCommunication/FRCComm.h"
#include "Task.h"
#include <dbgLib.h>
#include <stdio.h>
#include <taskLib.h>
#include <sysSymTbl.h>
#include "nivision.h"

#define DBG_DEMANGLE_PRINT_LEN MAX_SYS_SYM_LEN  /* Num chars of demangled names to print */

extern "C"
{
	extern char * cplusDemangle (char *source, char *dest, int32_t n);
}

void wpi_getLabel(UINT addr, char *label, int32_t *found)
{
	int pVal;
	SYM_TYPE pType;
	char name[MAX_SYS_SYM_LEN + 1];
  static const size_t kLabelSize = DBG_DEMANGLE_PRINT_LEN + 1 + 11;
	bzero(label, kLabelSize);

	if (symFindByValue(sysSymTbl, addr, name, &pVal, &pType) == OK)
	{
		cplusDemangle(name, label, kLabelSize - 11);
		if ((UINT)pVal != addr)
		{
			snprintf(label + strlen(label), kLabelSize - strlen(label), "+0x%04x", addr-pVal);
			if (found) *found = 2;
		}
		else
		{
			if (found) *found = 1;
		}
	}
	else
	{
		snprintf(label, kLabelSize, "0x%04x", addr);
		if (found) *found = 0;
	}
}
/*
static void wpiTracePrint(INSTR *caller, int32_t func, int32_t nargs, int32_t *args, int32_t taskId, BOOL isKernelAdrs)
{
	char buf [MAX_SYS_SYM_LEN * 2];
	int32_t ix;
	int32_t len = 0;
	len += snprintf (&buf [len], sizeof(buf) - len, "%s <%#010x>: ", wpi_getLabel((UINT)caller), (int32_t)caller);
	len += snprintf (&buf [len], sizeof(buf) - len, "%s <%#010x> (", wpi_getLabel((UINT)func), func);
	for (ix = 0; ix < nargs; ix++)
	{
		if (ix != 0) {
			len += snprintf (&buf [len], sizeof(buf) - len, ", ");
    }
		len += snprintf (&buf [len], sizeof(buf) - len, "%#x", args [ix]);
	}
	len += snprintf (&buf [len], sizeof(buf) - len, ")\n");

	printf(buf);
}
*/
static void wpiCleanTracePrint(INSTR *caller, int32_t func, int32_t nargs, int32_t *args, int32_t taskId, BOOL isKernelAdrs)
{
	char buf [MAX_SYS_SYM_LEN];
	int32_t ix;
	int32_t len = 0;
	int32_t nameFound = 0;
	int32_t params = 0;
	int32_t totalnargs = nargs;
  char funcName[DBG_DEMANGLE_PRINT_LEN + 1 + 11];
	wpi_getLabel((UINT)func, funcName, &nameFound);
	// Ignore names that are not exact symbol address matches.
	if (nameFound != 1) return;

	// Ignore internal function name matches.
	if (strncmp(funcName, "wpi_assert", 10) == 0) return;
	if (strncmp(funcName, "wpi_fatal", 9) == 0) return;
	if (strncmp(funcName, "wpi_selfTrace", 13) == 0) return;
	if (strncmp(funcName, "Error::Set", 10) == 0) return;
	if (strncmp(funcName, "ErrorBase::SetError", 19) == 0) return;
	if (strncmp(funcName, "Error::Report", 13) == 0) return;

	// Find the number of arguments in the name string.
	char *start = strchr(funcName, '(');
	char *end = strrchr(funcName, ')');
	if (start + 1 != end && start != NULL)
	{
		do
		{
			params++;
			if(strncmp(start+1, "bool", 4) == 0 || strncmp(start+2, "bool", 4) == 0)
			{
				totalnargs++;
			}
			start = strchr(start + 1, ',');
		}
		while(start < end && start != NULL);
	}
	char *funcNameEnd = strchr(funcName, '(');
	*funcNameEnd = 0;
	len += snprintf (buf + len, sizeof(buf) - len, funcName);

	// If this is a member function, print out the this pointer value.
	if (totalnargs - params == 1)
	{
		len += snprintf (&buf [len], sizeof(buf) - len, "<this=%#lx>", args [0]);
	}

	// Print out the argument values.
	len += snprintf (buf + len, sizeof(buf) - len, "(");
	for (ix = totalnargs - params; ix < nargs; ix++)
	{
		if (ix != totalnargs - params) {
			len += snprintf (&buf [len], sizeof(buf) - len, ", ");
    }
		len += snprintf (&buf [len], sizeof(buf) - len, "%#lx", args [ix]);
	}
	len += snprintf (buf + len, sizeof(buf) - len, ")\n");

	printf(buf);
}

extern "C"
{
	extern void trcStack(REG_SET* pRegs, FUNCPTR printRtn, int32_t tid);
}

static int32_t wpiStackTask(int32_t taskId)
{
  // Make sure it's suspended in spite of any scheduler weirdness or whatever.
  while (!taskIsSuspended(taskId)) {
	  taskDelay(1);
  }

	//tt(taskId);

	REG_SET regs;
	taskRegsGet(taskId, &regs);
	trcStack(&regs, (FUNCPTR) wpiCleanTracePrint, taskId);
	printf("\n");

	// The task should be resumed because it had to be suspended to get the stack trace.
	taskResume(taskId);
	return 0;
}

void wpi_selfTrace()
{
	int priority=100;
	taskPriorityGet(0, &priority);
	// Lower priority than the calling task.
	Task traceTask("StackTrace", (FUNCPTR)wpiStackTask, priority + 1);
	traceTask.Start(taskIdSelf());

	// Task to be traced must be suspended for the stack trace to work.
	taskSuspend(0);
}

static bool stackTraceEnabled = false;
static bool suspendOnAssertEnabled = false;

/**
 * Enable Stack trace after asserts.
 */
void wpi_stackTraceOnAssertEnable(bool enabled)
{
	stackTraceEnabled = enabled;
}

/**
 * Enable suspend on asssert.
 * If enabled, the user task will be suspended whenever an assert fails. This
 * will allow the user to attach to the task with the debugger and examine variables
 * around the failure.
 */
void wpi_suspendOnAssertEnabled(bool enabled)
{
	suspendOnAssertEnabled = enabled;
}

static void wpi_handleTracing()
{
	if (stackTraceEnabled)
	{
		printf("\n-----------<Stack Trace>----------------\n");
		wpi_selfTrace();
	}
	printf("\n");
}

/**
 * Assert implementation.
 * This allows breakpoints to be set on an assert.
 * The users don't call this, but instead use the wpi_assert macros in Utility.h.
 */
bool wpi_assert_impl(bool conditionValue, 
					 const char *conditionText,
					 const char *message,
					 const char *fileName,
					 uint32_t lineNumber, 
					 const char *funcName)
{
	if (!conditionValue)
	{   
		// Error string buffer
		char error[256];
				
		// If an error message was specified, include it
		// Build error string
		if(message != NULL) {
			snprintf(error, sizeof(error), "Assertion failed: \"%s\", \"%s\" failed in %s() in %s at line %ld\n",
							 message, conditionText, funcName, fileName, lineNumber);
		} else {
			snprintf(error, sizeof(error), "Assertion failed: \"%s\" in %s() in %s at line %ld\n",
							 conditionText, funcName, fileName, lineNumber);
		}
		
		// Print to console and send to remote dashboard
		printf("\n\n>>>>%s", error);
		setErrorData(error, strlen(error), 100);
		
		wpi_handleTracing();
		if (suspendOnAssertEnabled) taskSuspend(0);
	}
	return conditionValue;
}

/**
 * Common error routines for wpi_assertEqual_impl and wpi_assertNotEqual_impl
 * This should not be called directly; it should only be used by wpi_assertEqual_impl
 * and wpi_assertNotEqual_impl.
 */
void wpi_assertEqual_common_impl(int valueA,
					 	         int valueB,
					 	         const char *equalityType,
						         const char *message,
						         const char *fileName,
						         uint32_t lineNumber, 
						         const char *funcName)
{
	// Error string buffer
	char error[256];
			
	// If an error message was specified, include it
	// Build error string
	if(message != NULL) {
		snprintf(error, sizeof(error), "Assertion failed: \"%s\", \"%d\" %s \"%d\" in %s() in %s at line %ld\n",
						 message, valueA, equalityType, valueB, funcName, fileName, lineNumber);
	} else {
		snprintf(error, sizeof(error), "Assertion failed: \"%d\" %s \"%d\" in %s() in %s at line %ld\n",
						 valueA, equalityType, valueB, funcName, fileName, lineNumber);
	}
	
	// Print to console and send to remote dashboard
	printf("\n\n>>>>%s", error);
	setErrorData(error, strlen(error), 100);
	
	wpi_handleTracing();
	if (suspendOnAssertEnabled) taskSuspend(0);
}

/**
 * Assert equal implementation.
 * This determines whether the two given integers are equal. If not,
 * the value of each is printed along with an optional message string.
 * The users don't call this, but instead use the wpi_assertEqual macros in Utility.h.
 */
bool wpi_assertEqual_impl(int valueA,
					 	  int valueB,
						  const char *message,
						  const char *fileName,
						  uint32_t lineNumber, 
						  const char *funcName)
{
	if(!(valueA == valueB))
	{
		wpi_assertEqual_common_impl(valueA, valueB, "!=", message, fileName, lineNumber, funcName);
	}
	return valueA == valueB;
}

/**
 * Assert not equal implementation.
 * This determines whether the two given integers are equal. If so,
 * the value of each is printed along with an optional message string.
 * The users don't call this, but instead use the wpi_assertNotEqual macros in Utility.h.
 */
bool wpi_assertNotEqual_impl(int valueA,
					 	     int valueB,
						     const char *message,
						     const char *fileName,
						     uint32_t lineNumber, 
						     const char *funcName)
{
	if(!(valueA != valueB))
	{
		wpi_assertEqual_common_impl(valueA, valueB, "==", message, fileName, lineNumber, funcName);
	}
	return valueA != valueB;
}
