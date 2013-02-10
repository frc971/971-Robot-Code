/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/
#include "CInterfaces/CTimer.h"
#include <stdio.h>
#include "Utility.h"

static Timer *timers[kMaxTimers];
static bool initialized = false;

/**
 * Allocate the resources for a timer object
 * Timers are allocated in an array and indexed with the "index" parameter. There
 * can be up to 10 timer objects in use at any one time. Deleting a timer object
 * frees up it's slot and resources.
 *
 * @param index The index of this timer object.
 */
static Timer *AllocateTimer(UINT32 index)
{
	if (!initialized)
	{
		for (unsigned i = 0; i < kMaxTimers; i++)
			timers[i] = NULL;
		initialized = true;
	}
	if (index == 0 || index >= kMaxTimers)
	{
		printf("Timer index out of range [1, %d]: %d\n", kMaxTimers, index);
		return NULL;
	}
	Timer *timer = timers[index - 1];
	if (timer == NULL)
	{
		timer = new Timer();
		timers[index - 1] = timer;
	}
	return timer;
}

/**
 * Reset the timer by setting the time to 0.
 *
 * Make the timer startTime the current time so new requests will be relative now
 * @param index The index of this timer object.
 */
void ResetTimer(UINT32 index)
{
	Timer *timer = AllocateTimer(index);
	if (timer != NULL)
	{
		timer->Reset();
	}
}

/**
 * Start the timer running.
 * Just set the running flag to true indicating that all time requests should be
 * relative to the system clock.
 *
 * @param index The index of this timer object.
 */
void StartTimer(UINT32 index)
{
	Timer *timer = AllocateTimer(index);
	if (timer != NULL)
	{
		timer->Start();
	}
}

/**
 * Stop the timer.
 * This computes the time as of now and clears the running flag, causing all
 * subsequent time requests to be read from the accumulated time rather than
 * looking at the system clock.
 *
 * @param index The index of this timer object.
 */
void StopTimer(UINT32 index)
{
	Timer *timer = AllocateTimer(index);
	if (timer != NULL)
	{
		timer->Stop();
	}
}

/**
 * Get the current time from the timer. If the clock is running it is derived from
 * the current system clock the start time stored in the timer class. If the clock
 * is not running, then return the time when it was last stopped.
 *
 * @param index The timer index being used
 * @return unsigned Current time value for this timer in seconds
 */
double GetTimer(UINT32 index)
{
	Timer *timer = AllocateTimer(index);
	if (timer != NULL)
	{
		return timer->Get();
	}
	else
		return 0.0;
}

/**
 * Free the resources associated with this timer object
 *
 * @param index The index of this timer object.
 */
void DeleteTimer(UINT32 index)
{
	if (index >= 1 && index <= kMaxTimers)
	{
		delete timers[index - 1];
		timers[index - 1] = NULL;
	}
}
