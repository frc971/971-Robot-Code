/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc971/wpilib/ahal/SensorBase.h"

#include "FRC_NetworkCommunication/LoadOut.h"
#include "frc971/wpilib/ahal/WPIErrors.h"
#include "hal/AnalogInput.h"
#include "hal/AnalogOutput.h"
#include "hal/DIO.h"
#include "hal/HAL.h"
#include "hal/PWM.h"
#include "hal/Ports.h"
#include "hal/PowerDistribution.h"
#include "hal/Relay.h"

namespace frc {

const int kDigitalChannels = HAL_GetNumDigitalChannels();
const int kAnalogInputs = HAL_GetNumAnalogInputs();
const int kPwmChannels = HAL_GetNumPWMChannels();
const int kRelayChannels = HAL_GetNumRelayHeaders();
const int kCTREPDPChannels = HAL_GetNumCTREPDPChannels();
const int kREVPDPChannels = HAL_GetNumREVPDHChannels();

/**
 * Check that the digital channel number is valid.
 *
 * Verify that the channel number is one of the legal channel numbers. Channel
 * numbers are 1-based.
 *
 * @return Digital channel is valid
 */
bool CheckDigitalChannel(int channel) { return HAL_CheckDIOChannel(channel); }

/**
 * Check that the relay channel number is valid.
 *
 * Verify that the channel number is one of the legal channel numbers. Channel
 * numbers are 0-based.
 *
 * @return Relay channel is valid
 */
bool CheckRelayChannel(int channel) { return HAL_CheckRelayChannel(channel); }

/**
 * Check that the digital channel number is valid.
 *
 * Verify that the channel number is one of the legal channel numbers. Channel
 * numbers are 1-based.
 *
 * @return PWM channel is valid
 */
bool CheckPWMChannel(int channel) { return HAL_CheckPWMChannel(channel); }

/**
 * Check that the analog input number is value.
 *
 * Verify that the analog input number is one of the legal channel numbers.
 * Channel numbers are 0-based.
 *
 * @return Analog channel is valid
 */
bool CheckAnalogInputChannel(int channel) {
  return HAL_CheckAnalogInputChannel(channel);
}

/**
 * Check that the analog output number is valid.
 *
 * Verify that the analog output number is one of the legal channel numbers.
 * Channel numbers are 0-based.
 *
 * @return Analog channel is valid
 */
bool CheckAnalogOutputChannel(int channel) {
  return HAL_CheckAnalogOutputChannel(channel);
}

/**
 * Verify that the power distribution channel number is within limits.
 *
 * @return PDP channel is valid
 */
bool CheckPDPChannel(int channel, HAL_PowerDistributionType type) {
  return HAL_CheckPowerDistributionModule(channel, type);
}

}  // namespace frc
