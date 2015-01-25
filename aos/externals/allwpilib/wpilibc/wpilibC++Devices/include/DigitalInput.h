/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/
#pragma once

#include <array>

#include "DigitalSource.h"

class Encoder;
class Counter;
class DigitalGlitchFilter;

/**
 * Class to read a digital input.
 * This class will read digital inputs and return the current value on the channel. Other devices
 * such as encoders, gear tooth sensors, etc. that are implemented elsewhere will automatically
 * allocate digital inputs and outputs as required. This class is only for devices like switches
 * etc. that aren't implemented anywhere else.
 */
class DigitalInput : public DigitalSource
{
public:
	explicit DigitalInput(uint32_t channel);
	virtual ~DigitalInput();
	virtual bool Get();
	uint32_t GetChannel();

	// Digital Source Interface
	virtual uint32_t GetChannelForRouting();
	virtual uint32_t GetModuleForRouting();
	virtual bool GetAnalogTriggerForRouting();

private:
	void InitDigitalInput(uint32_t channel);
	uint32_t m_channel;
	bool m_lastValue;

  friend DigitalGlitchFilter;
};

/**
 * Class to enable glitch filtering on a set of digital inputs.
 * This class will manage adding and removing digital inputs from a FPGA glitch
 * filter. The filter lets the user configure the time that an input must remain
 * high or low before it is classified as high or low.
 */
class DigitalGlitchFilter : public ErrorBase {
 public:
  DigitalGlitchFilter();
  ~DigitalGlitchFilter();

  /**
   * Assigns the DigitalSource to this glitch filter.
   *
   * @param input The DigitalSource to add.
   */
  void Add(DigitalSource *input);

  /**
   * Assigns the Encoder to this glitch filter.
   *
   * @param input The Encoder to add.
   */
  void Add(Encoder *input);

  /**
   * Assigns the Counter to this glitch filter.
   *
   * @param input The Counter to add.
   */
  void Add(Counter *input);

  /**
   * Removes a digital input from this filter.
   *
   * Removes the DigitalSource from this glitch filter and re-assigns it to the
   * default filter.
   *
   * @param input The DigitalSource to remove.
   */
  void Remove(DigitalSource *input);
  void Remove(Encoder *input);
  void Remove(Counter *input);

  /**
   * Sets the number of cycles that the input must not change state for.
   *
   * @param fpga_cycles The number of FPGA cycles.
   */
  void SetPeriodCycles(uint32_t fpga_cycles);

  /**
   * Sets the number of nanoseconds that the input must not change state for.
   *
   * @param nanoseconds The number of nanoseconds.
   */
  void SetPeriodNanoSeconds(uint64_t nanoseconds);

  /**
   * Gets the number of cycles that the input must not change state for.
   *
   * @return The number of cycles.
   */
  uint32_t GetPeriodCycles();
  /**
   * Gets the number of nanoseconds that the input must not change state for.
   *
   * @return The number of nanoseconds.
   */
  uint64_t GetPeriodNanoSeconds();

 private:
  int channel_index_;
  static Mutex mutex_;
  static ::std::array<int, 3> filter_allocated_;
};
