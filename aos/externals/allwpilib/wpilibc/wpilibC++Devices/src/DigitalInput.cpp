/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include <algorithm>
#include <array>

#include "DigitalInput.h"
//#include "NetworkCommunication/UsageReporting.h"
#include "Resource.h"
#include "WPIErrors.h"
#include "Encoder.h"
#include "Counter.h"

::std::array<int, 3> DigitalGlitchFilter::filter_allocated_ = {
    {false, false, false}};
Mutex DigitalGlitchFilter::mutex_;

/**
 * Create an instance of a DigitalInput.
 * Creates a digital input given a channel. Common creation routine for all
 * constructors.
 */
void DigitalInput::InitDigitalInput(uint32_t channel)
{
	char buf[64];

	if (!CheckDigitalChannel(channel))
	{
		snprintf(buf, 64, "Digital Channel %d", channel);
		wpi_setWPIErrorWithContext(ChannelIndexOutOfRange, buf);
		return;
	}
	m_channel = channel;

	int32_t status = 0;
	allocateDIO(m_digital_ports[channel], true, &status);
	wpi_setErrorWithContext(status, getHALErrorMessage(status));

	HALReport(HALUsageReporting::kResourceType_DigitalInput, channel);
}

/**
 * Create an instance of a Digital Input class.
 * Creates a digital input given a channel.
 *
 * @param channel The DIO channel 0-9 are on-board, 10-25 are on the MXP port
 */
DigitalInput::DigitalInput(uint32_t channel)
{
	InitDigitalInput(channel);
}

/**
 * Free resources associated with the Digital Input class.
 */
DigitalInput::~DigitalInput()
{
	if (StatusIsFatal()) return;
	if (m_interrupt != NULL)
	{
		int32_t status = 0;
		cleanInterrupts(m_interrupt, &status);
		wpi_setErrorWithContext(status, getHALErrorMessage(status));
		m_interrupt = NULL;
		m_interrupts->Free(m_interruptIndex);
	}

	int32_t status = 0;
	freeDIO(m_digital_ports[m_channel], &status);
	wpi_setErrorWithContext(status, getHALErrorMessage(status));
}

/**
 * Get the value from a digital input channel.
 * Retrieve the value of a single digital input channel from the FPGA.
 */
bool DigitalInput::Get()
{
	int32_t status = 0;
	bool value = getDIO(m_digital_ports[m_channel], &status);
	wpi_setErrorWithContext(status, getHALErrorMessage(status));
	return value;
}

/**
 * @return The GPIO channel number that this object represents.
 */
uint32_t DigitalInput::GetChannel()
{
	return m_channel;
}

/**
 * @return The value to be written to the channel field of a routing mux.
 */
uint32_t DigitalInput::GetChannelForRouting()
{
	return GetChannel();
}

/**
 * @return The value to be written to the module field of a routing mux.
 */
uint32_t DigitalInput::GetModuleForRouting()
{
	return 0;
}

/**
 * @return The value to be written to the analog trigger field of a routing mux.
 */
bool DigitalInput::GetAnalogTriggerForRouting()
{
	return false;
}

DigitalGlitchFilter::DigitalGlitchFilter() : channel_index_(-1) {
  ::std::unique_lock<Mutex> sync(DigitalGlitchFilter::mutex_);
  auto index =
      ::std::find(filter_allocated_.begin(), filter_allocated_.end(), false);
  if (index != filter_allocated_.end()) {
    channel_index_ = index - filter_allocated_.begin();
    *index = true;
  }
}

DigitalGlitchFilter::~DigitalGlitchFilter() {
  ::std::unique_lock<Mutex> sync(DigitalGlitchFilter::mutex_);
  filter_allocated_[channel_index_] = false;
}

void DigitalGlitchFilter::Add(DigitalSource *input) {
  int32_t status = 0;
  setFilterSelect(input->m_digital_ports[input->GetChannelForRouting()],
                  channel_index_ + 1, &status);
  wpi_setErrorWithContext(status, getHALErrorMessage(status));

  HALReport(HALUsageReporting::kResourceType_DigitalInput,
            input->GetChannelForRouting());
}

void DigitalGlitchFilter::Remove(DigitalSource *input) {
	int32_t status = 0;
	setFilterSelect(input->m_digital_ports[input->GetChannelForRouting()], 0, &status);
	wpi_setErrorWithContext(status, getHALErrorMessage(status));

	HALReport(HALUsageReporting::kResourceType_DigitalInput, input->GetChannelForRouting());
}

void DigitalGlitchFilter::SetPeriodCycles(uint32_t fpga_cycles) {
  int32_t status = 0;
  setFilterPeriod(channel_index_, fpga_cycles, &status);
  wpi_setErrorWithContext(status, getHALErrorMessage(status));

  HALReport(HALUsageReporting::kResourceType_DigitalGlitchFilter,
            channel_index_);
}

void DigitalGlitchFilter::SetPeriodNanoSeconds(uint64_t nanoseconds) {
  int32_t status = 0;
  uint32_t fpga_cycles =
      nanoseconds * kSystemClockTicksPerMicrosecond / 4 / 1000;
  setFilterPeriod(channel_index_, fpga_cycles, &status);

  wpi_setErrorWithContext(status, getHALErrorMessage(status));

  HALReport(HALUsageReporting::kResourceType_DigitalGlitchFilter,
            channel_index_);
}

uint32_t DigitalGlitchFilter::GetPeriodCycles() {
  int32_t status = 0;
  uint32_t fpga_cycles = getFilterPeriod(channel_index_, &status);

  wpi_setErrorWithContext(status, getHALErrorMessage(status));

  HALReport(HALUsageReporting::kResourceType_DigitalGlitchFilter,
            channel_index_);
  return fpga_cycles;
}

uint64_t DigitalGlitchFilter::GetPeriodNanoSeconds() {
  int32_t status = 0;
  uint32_t fpga_cycles = getFilterPeriod(channel_index_, &status);

  wpi_setErrorWithContext(status, getHALErrorMessage(status));

  HALReport(HALUsageReporting::kResourceType_DigitalGlitchFilter,
            channel_index_);
  return static_cast<uint64_t>(fpga_cycles) * 1000l /
         static_cast<uint64_t>(kSystemClockTicksPerMicrosecond / 4);
}

void DigitalGlitchFilter::Add(Encoder *input) {
  Add(input->m_aSource);
  if (StatusIsFatal()) {
    return;
  }
  Add(input->m_bSource);
}

void DigitalGlitchFilter::Add(Counter *input) {
  Add(input->m_upSource);
  if (StatusIsFatal()) {
    return;
  }
  Add(input->m_downSource);
}

void DigitalGlitchFilter::Remove(Encoder *input) {
  Remove(input->m_aSource);
  if (StatusIsFatal()) {
    return;
  }
  Remove(input->m_bSource);
}

void DigitalGlitchFilter::Remove(Counter *input) {
  Remove(input->m_upSource);
  if (StatusIsFatal()) {
    return;
  }
  Remove(input->m_downSource);
}
