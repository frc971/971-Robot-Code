/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc971/wpilib/ahal/AnalogInput.h"
#include "hal/AnalogInput.h"

#include <sstream>

#include "hal/HAL.h"
#include "hal/Ports.h"
#include "frc971/wpilib/ahal/WPIErrors.h"

using namespace frc;

/**
 * Construct an analog input.
 *
 * @param channel The channel number on the roboRIO to represent. 0-3 are
 *                on-board 4-7 are on the MXP port.
 */
AnalogInput::AnalogInput(int channel) {
  std::stringstream buf;
  buf << "Analog Input " << channel;

  if (!CheckAnalogInputChannel(channel)) {
    wpi_setWPIErrorWithContext(ChannelIndexOutOfRange, buf.str());
    return;
  }

  m_channel = channel;

  HAL_PortHandle port = HAL_GetPort(channel);
  int32_t status = 0;
  m_port = HAL_InitializeAnalogInputPort(port, &status);
  if (status != 0) {
    wpi_setErrorWithContextRange(status, 0, HAL_GetNumAnalogInputs(), channel,
                                 HAL_GetErrorMessage(status));
    m_channel = std::numeric_limits<int>::max();
    m_port = HAL_kInvalidHandle;
    return;
  }

  HAL_Report(HALUsageReporting::kResourceType_AnalogChannel, channel);
}

/**
 * Channel destructor.
 */
AnalogInput::~AnalogInput() {
  HAL_FreeAnalogInputPort(m_port);
  m_port = HAL_kInvalidHandle;
}

/**
 * Get a sample straight from this channel.
 *
 * The sample is a 12-bit value representing the 0V to 5V range of the A/D
 * converter in the module.  The units are in A/D converter codes.  Use
 * GetVoltage() to get the analog value in calibrated units.
 *
 * @return A sample straight from this channel.
 */
int AnalogInput::GetValue() const {
  if (StatusIsFatal()) return 0;
  int32_t status = 0;
  int value = HAL_GetAnalogValue(m_port, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  return value;
}

/**
 * Get a sample from the output of the oversample and average engine for this
 * channel.
 *
 * The sample is 12-bit + the bits configured in SetOversampleBits().
 * The value configured in SetAverageBits() will cause this value to be averaged
 * 2**bits number of samples.
 * This is not a sliding window.  The sample will not change until
 * 2**(OversampleBits + AverageBits) samples
 * have been acquired from the module on this channel.
 * Use GetAverageVoltage() to get the analog value in calibrated units.
 *
 * @return A sample from the oversample and average engine for this channel.
 */
int AnalogInput::GetAverageValue() const {
  if (StatusIsFatal()) return 0;
  int32_t status = 0;
  int value = HAL_GetAnalogAverageValue(m_port, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  return value;
}

/**
 * Get a scaled sample straight from this channel.
 *
 * The value is scaled to units of Volts using the calibrated scaling data from
 * GetLSBWeight() and GetOffset().
 *
 * @return A scaled sample straight from this channel.
 */
double AnalogInput::GetVoltage() const {
  if (StatusIsFatal()) return 0.0;
  int32_t status = 0;
  double voltage = HAL_GetAnalogVoltage(m_port, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  return voltage;
}

/**
 * Get a scaled sample from the output of the oversample and average engine for
 * this channel.
 *
 * The value is scaled to units of Volts using the calibrated scaling data from
 * GetLSBWeight() and GetOffset().
 * Using oversampling will cause this value to be higher resolution, but it will
 * update more slowly.
 * Using averaging will cause this value to be more stable, but it will update
 * more slowly.
 *
 * @return A scaled sample from the output of the oversample and average engine
 * for this channel.
 */
double AnalogInput::GetAverageVoltage() const {
  if (StatusIsFatal()) return 0.0;
  int32_t status = 0;
  double voltage = HAL_GetAnalogAverageVoltage(m_port, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  return voltage;
}

/**
 * Get the factory scaling least significant bit weight constant.
 *
 * Volts = ((LSB_Weight * 1e-9) * raw) - (Offset * 1e-9)
 *
 * @return Least significant bit weight.
 */
int AnalogInput::GetLSBWeight() const {
  if (StatusIsFatal()) return 0;
  int32_t status = 0;
  int lsbWeight = HAL_GetAnalogLSBWeight(m_port, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  return lsbWeight;
}

/**
 * Get the factory scaling offset constant.
 *
 * Volts = ((LSB_Weight * 1e-9) * raw) - (Offset * 1e-9)
 *
 * @return Offset constant.
 */
int AnalogInput::GetOffset() const {
  if (StatusIsFatal()) return 0;
  int32_t status = 0;
  int offset = HAL_GetAnalogOffset(m_port, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  return offset;
}

/**
 * Get the channel number.
 *
 * @return The channel number.
 */
int AnalogInput::GetChannel() const {
  if (StatusIsFatal()) return 0;
  return m_channel;
}

/**
 * Set the number of averaging bits.
 *
 * This sets the number of averaging bits. The actual number of averaged samples
 * is 2^bits.
 * Use averaging to improve the stability of your measurement at the expense of
 * sampling rate.
 * The averaging is done automatically in the FPGA.
 *
 * @param bits Number of bits of averaging.
 */
void AnalogInput::SetAverageBits(int bits) {
  if (StatusIsFatal()) return;
  int32_t status = 0;
  HAL_SetAnalogAverageBits(m_port, bits, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
}

/**
 * Get the number of averaging bits previously configured.
 *
 * This gets the number of averaging bits from the FPGA. The actual number of
 * averaged samples is 2^bits. The averaging is done automatically in the FPGA.
 *
 * @return Number of bits of averaging previously configured.
 */
int AnalogInput::GetAverageBits() const {
  int32_t status = 0;
  int averageBits = HAL_GetAnalogAverageBits(m_port, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  return averageBits;
}

/**
 * Set the number of oversample bits.
 *
 * This sets the number of oversample bits. The actual number of oversampled
 * values is 2^bits. Use oversampling to improve the resolution of your
 * measurements at the expense of sampling rate. The oversampling is done
 * automatically in the FPGA.
 *
 * @param bits Number of bits of oversampling.
 */
void AnalogInput::SetOversampleBits(int bits) {
  if (StatusIsFatal()) return;
  int32_t status = 0;
  HAL_SetAnalogOversampleBits(m_port, bits, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
}

/**
 * Get the number of oversample bits previously configured.
 *
 * This gets the number of oversample bits from the FPGA. The actual number of
 * oversampled values is 2^bits. The oversampling is done automatically in the
 * FPGA.
 *
 * @return Number of bits of oversampling previously configured.
 */
int AnalogInput::GetOversampleBits() const {
  if (StatusIsFatal()) return 0;
  int32_t status = 0;
  int oversampleBits = HAL_GetAnalogOversampleBits(m_port, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  return oversampleBits;
}

/**
 * Set the sample rate per channel for all analog channels.
 *
 * The maximum rate is 500kS/s divided by the number of channels in use.
 * This is 62500 samples/s per channel.
 *
 * @param samplesPerSecond The number of samples per second.
 */
void AnalogInput::SetSampleRate(double samplesPerSecond) {
  int32_t status = 0;
  HAL_SetAnalogSampleRate(samplesPerSecond, &status);
  wpi_setGlobalErrorWithContext(status, HAL_GetErrorMessage(status));
}

/**
 * Get the current sample rate for all channels
 *
 * @return Sample rate.
 */
double AnalogInput::GetSampleRate() {
  int32_t status = 0;
  double sampleRate = HAL_GetAnalogSampleRate(&status);
  wpi_setGlobalErrorWithContext(status, HAL_GetErrorMessage(status));
  return sampleRate;
}
