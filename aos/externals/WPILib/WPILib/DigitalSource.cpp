/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "DigitalSource.h"
#include "Resource.h"
#include "WPIErrors.h"

Resource *interruptsResource = NULL;

DigitalSource::DigitalSource()
{
	Resource::CreateResourceObject(&interruptsResource, tInterrupt::kNumSystems);
}

/**
 * DigitalSource destructor.
 */
DigitalSource::~DigitalSource()
{
	if (m_manager != NULL)
	{
		delete m_manager;
		delete m_interrupt;
		interruptsResource->Free(m_interruptIndex);
	}
}

/**
 * Request interrupts asynchronously on this digital input.
 * @param handler The address of the interrupt handler function of type tInterruptHandler that
 * will be called whenever there is an interrupt on the digitial input port.
 * Request interrupts in synchronus mode where the user program interrupt handler will be
 * called when an interrupt occurs.
 * The default is interrupt on rising edges only.
 */
void DigitalSource::RequestInterrupts(tInterruptHandler handler, void *param)
{
	if (StatusIsFatal()) return;
	UINT32 index = interruptsResource->Allocate("Async Interrupt");
	if (index == ~0ul)
	{
		CloneError(interruptsResource);
		return;
	}
	m_interruptIndex = index;

	 // Creates a manager too
	AllocateInterrupts(false);

	tRioStatusCode localStatus = NiFpga_Status_Success;
	m_interrupt->writeConfig_WaitForAck(false, &localStatus);
	m_interrupt->writeConfig_Source_AnalogTrigger(GetAnalogTriggerForRouting(), &localStatus);
	m_interrupt->writeConfig_Source_Channel(GetChannelForRouting(), &localStatus);
	m_interrupt->writeConfig_Source_Module(GetModuleForRouting(), &localStatus);
	SetUpSourceEdge(true, false);

	m_manager->registerHandler(handler, param, &localStatus);
	wpi_setError(localStatus);
}

/**
 * Request interrupts synchronously on this digital input.
 * Request interrupts in synchronus mode where the user program will have to explicitly
 * wait for the interrupt to occur.
 * The default is interrupt on rising edges only.
 */
void DigitalSource::RequestInterrupts()
{
	if (StatusIsFatal()) return;
	UINT32 index = interruptsResource->Allocate("Sync Interrupt");
	if (index == ~0ul)
	{
		CloneError(interruptsResource);
		return;
	}
	m_interruptIndex = index;

	AllocateInterrupts(true);

	tRioStatusCode localStatus = NiFpga_Status_Success;
	m_interrupt->writeConfig_Source_AnalogTrigger(GetAnalogTriggerForRouting(), &localStatus);
	m_interrupt->writeConfig_Source_Channel(GetChannelForRouting(), &localStatus);
	m_interrupt->writeConfig_Source_Module(GetModuleForRouting(), &localStatus);
	SetUpSourceEdge(true, false);
	wpi_setError(localStatus);
}

void DigitalSource::SetUpSourceEdge(bool risingEdge, bool fallingEdge)
{
	if (StatusIsFatal()) return;
	if (m_interrupt == NULL)
	{
		wpi_setWPIErrorWithContext(NullParameter, "You must call RequestInterrupts before SetUpSourceEdge");
		return;
	}
	tRioStatusCode localStatus = NiFpga_Status_Success;
	if (m_interrupt != NULL)
	{
		m_interrupt->writeConfig_RisingEdge(risingEdge, &localStatus);
		m_interrupt->writeConfig_FallingEdge(fallingEdge, &localStatus);
	}
	wpi_setError(localStatus);
}
