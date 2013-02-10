/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef SYNCHRONIZED_H
#define SYNCHRONIZED_H

#include <semLib.h>

#include "Base.h"

#define CRITICAL_REGION(s) { Synchronized _sync(s);
#define END_REGION }

class Synchronized;

/**
 * Wrap a raw vxworks semaphore (SEM_ID).
 * This class wraps a raw vxworks semaphore so that it is created and destroyed
 * with C++ constructors and destructors.
 * This class is safe to use in static variables because it does not depend on
 * any other C++ static constructors or destructors.
 */
class Semaphore
{
public:
	Semaphore();
	~Semaphore();
	int Take();
	int Give();
private:
	SEM_ID m_semaphore;
	SEM_ID get();

	// TODO somebody should change Synchronized to not support raw SEM_IDs and
	// instead use Take() and Give() directly so this can go away
	friend class Synchronized;
	DISALLOW_COPY_AND_ASSIGN(Semaphore);
};

/**
 * Provide easy support for critical regions.
 * A critical region is an area of code that is always executed under mutual exclusion. Only
 * one task can be executing this code at any time. The idea is that code that manipulates data
 * that is shared between two or more tasks has to be prevented from executing at the same time
 * otherwise a race condition is possible when both tasks try to update the data. Typically
 * semaphores are used to ensure only single task access to the data.
 * Synchronized objects are a simple wrapper around semaphores to help ensure that semaphores
 * are always signaled (semGive) after a wait (semTake).
 */
class Synchronized
{
public:
	explicit Synchronized(SEM_ID);
	explicit Synchronized(Semaphore&);
	virtual ~Synchronized();
private:
	SEM_ID m_semaphore;
	DISALLOW_COPY_AND_ASSIGN(Synchronized);
};

#endif
