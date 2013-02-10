#include "VxWorks.h"
#include "CInterfaces/CCounter.h"
#include "Counter.h"
#include "DigitalModule.h"

static Counter* counters[SensorBase::kDigitalModules][SensorBase::kDigitalChannels];
static bool initialized = false;

/**
 * Allocate the resource for a counter
 * Allocate the underlying Counter object and the resources associated with the
 * slot and channel
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The channel of the digital input used with this counter
 */
static Counter *AllocateCounter(UINT8 moduleNumber, UINT32 channel)
{
	if (!initialized)
	{
		for (unsigned i = 0; i < SensorBase::kDigitalModules; i++)
			for (unsigned j = 0; j < SensorBase::kDigitalChannels; j++)
				counters[i][j] = NULL;
		initialized = true;
	}
	if (SensorBase::CheckDigitalModule(moduleNumber) && SensorBase::CheckDigitalChannel(channel))
	{
		UINT32 slotOffset = moduleNumber - 1;
		if (counters[slotOffset][channel - 1] == NULL)
			counters[slotOffset][channel - 1] = new Counter(moduleNumber, channel);
		return counters[slotOffset][channel - 1];
	}
	else
		return NULL;
}

/**
 * Allocate the resource for a counter
 * Allocate the underlying Counter object and the resources associated with the
 * slot and channel
 *
 * @param channel The channel of the digital input used with this counter
 */
static Counter *AllocateCounter(UINT32 channel)
{
	return AllocateCounter(DigitalModule::GetDefaultDigitalModule(), channel);
}

/**
 * Start the Counter counting.
 * This enables the counter and it starts accumulating counts from the associated
 * input channel. The counter value is not reset on starting, and still has the previous value.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The channel of the digital input used with this counter
 */
void StartCounter(UINT8 moduleNumber, UINT32 channel)
{
	Counter *counter = AllocateCounter(moduleNumber, channel);
	if (counter != NULL)
		counter->Start();
}

/**
 * Start the Counter counting.
 * This enables the counter and it starts accumulating counts from the associated
 * input channel. The counter value is not reset on starting, and still has the previous value.
 *
 * @param channel The channel of the digital input used with this counter
 */
void StartCounter(UINT32 channel)
{
	Counter *counter = AllocateCounter(channel);
	if (counter != NULL)
		counter->Start();
}

/**
 * Read the current counter value.
 * Read the value at this instant. It may still be running, so it reflects the current value. Next
 * time it is read, it might have a different value.

 * @param channel The channel of the digital input used with this counter
 */
INT32 GetCounter(UINT32 channel)
{
	Counter *counter = AllocateCounter(channel);
	if (counter != NULL)
		return counter->Get();
	else
		return 0;
}

/**
 * Read the current counter value.
 * Read the value at this instant. It may still be running, so it reflects the current value. Next
 * time it is read, it might have a different value.

 * @param slot The slot the digital module is plugged into
 * @param channel The channel of the digital input used with this counter
 */
INT32 GetCounter(UINT8 moduleNumber, UINT32 channel)
{
	Counter *counter = AllocateCounter(moduleNumber, channel);
	if (counter != NULL)
		return counter->Get();
	else
		return 0;
}

/**
 * Reset the Counter to zero.
 * Set the counter value to zero. This doesn't effect the running state of the counter, just sets
 * the current value to zero.
 * @param channel The channel of the digital input used with this counter
 */
void ResetCounter(UINT32 channel)
{
	Counter *counter = AllocateCounter(channel);
	if (counter != NULL)
		counter->Reset();
}

/**
 * Reset the Counter to zero.
 * Set the counter value to zero. This doesn't effect the running state of the counter, just sets
 * the current value to zero.
 * @param slot The slot the digital module is plugged into
 * @param channel The channel of the digital input used with this counter
 */
void ResetCounter(UINT8 moduleNumber, UINT32 channel)
{
	Counter *counter = AllocateCounter(moduleNumber, channel);
	if (counter != NULL)
		counter->Reset();
}

/**
 * Stop the Counter.
 * Stops the counting but doesn't effect the current value.
 * @param slot The slot the digital module is plugged into
 * @param channel The channel of the digital input used with this counter
 */
void StopCounter(UINT8 moduleNumber, UINT32 channel)
{
	Counter *counter = AllocateCounter(moduleNumber, channel);
	if (counter != NULL)
		counter->Stop();
}

/**
 * Stop the Counter.
 * Stops the counting but doesn't effect the current value.
 * @param channel The channel of the digital input used with this counter
 */
void StopCounter(UINT32 channel)
{
	Counter *counter = AllocateCounter(channel);
	if (counter != NULL)
		counter->Stop();
}

/*
 * Get the Period of the most recent count.
 * Returns the time interval of the most recent count. This can be used for velocity calculations
 * to determine shaft speed.
 * @param slot The slot the digital module is plugged into
 * @param channel The channel of the digital input used with this counter
 * @returns The period of the last two pulses in units of seconds.
 */
double GetCounterPeriod(UINT8 moduleNumber, UINT32 channel)
{
	Counter *counter = AllocateCounter(moduleNumber, channel);
	if (counter != NULL)
		return counter->GetPeriod();
	else
		return 0;
}

/*
 * Get the Period of the most recent count.
 * Returns the time interval of the most recent count. This can be used for velocity calculations
 * to determine shaft speed.
 * @param channel The channel of the digital input used with this counter
 * @returns The period of the last two pulses in units of seconds.
 */
double GetCounterPeriod(UINT32 channel)
{
	Counter *counter = AllocateCounter(channel);
	if (counter != NULL)
		return counter->GetPeriod();
	else
		return 0;
}

/**
 * Delete the resources associated with this counter.
 * The resources including the underlying object are deleted for this counter.
 * @param slot The slot the digital module is plugged into
 * @param channel The channel of the digital input used with this counter
 */
void DeleteCounter(UINT8 moduleNumber, UINT32 channel)
{
	if (SensorBase::CheckDigitalModule(moduleNumber) && SensorBase::CheckDigitalChannel(channel))
	{
		UINT32 slotOffset = moduleNumber - 1;
		delete counters[slotOffset][channel - 1];
		counters[slotOffset][channel - 1] = NULL;
	}
}

/**
 * Delete the resources associated with this counter.
 * The resources including the underlying object are deleted for this counter.
 * @param channel The channel of the digital input used with this counter
 */
void DeleteCounter(UINT32 channel)
{
	DeleteCounter(SensorBase::GetDefaultDigitalModule(), channel);
}

/*******************************************************************************
 * Alternative interface to counter
*******************************************************************************/

CounterObject CreateCounter(UINT32 channel) 
{
	return (CounterObject) new Counter(channel);
}

CounterObject CreateCounter(UINT8 moduleNumber, UINT32 channel) 
{
	return (CounterObject) new Counter(moduleNumber,channel);	
}

void StartCounter(CounterObject o)
{
	((Counter *) o)->Start();
}

INT32 GetCounter(CounterObject o)
{
	return 	((Counter *) o)->Get();
}

void ResetCounter(CounterObject o)
{
	((Counter *) o)->Reset();	
}

void StopCounter(CounterObject o)
{
	((Counter *) o)->Stop();
}

double GetCounterPeriod(CounterObject o)
{
	return 	((Counter *) o)->GetPeriod();	
}

void DeleteCounter(CounterObject o)
{
	delete (Counter *) o;
}
