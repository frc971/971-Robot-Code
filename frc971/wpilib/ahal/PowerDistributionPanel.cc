/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2014-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc971/wpilib/ahal/PowerDistributionPanel.h"

#include <sstream>

#include "hal/HAL.h"
#include "hal/PDP.h"
#include "hal/Ports.h"
#include "frc971/wpilib/ahal/WPIErrors.h"

using namespace frc;
#define WPI_LIB_FATAL_ERROR(tag, msg)

PowerDistributionPanel::PowerDistributionPanel() : PowerDistributionPanel(0) {}

/**
 * Initialize the PDP.
 */
PowerDistributionPanel::PowerDistributionPanel(int module) {
  int32_t status = 0;
  m_handle = HAL_InitializePDP(module, &status);
  if (status != 0) {
    return;
  }
}

/**
 * Query the input voltage of the PDP.
 *
 * @return The voltage of the PDP in volts
 */
double PowerDistributionPanel::GetVoltage() const {
  int32_t status = 0;

  double voltage = HAL_GetPDPVoltage(m_handle, &status);

  if (status) {
    WPI_LIB_FATAL_ERROR(Timeout, "");
  }

  return voltage;
}

/**
 * Query the temperature of the PDP.
 *
 * @return The temperature of the PDP in degrees Celsius
 */
double PowerDistributionPanel::GetTemperature() const {
  int32_t status = 0;

  double temperature = HAL_GetPDPTemperature(m_handle, &status);

  if (status) {
    WPI_LIB_FATAL_ERROR(Timeout, "");
  }

  return temperature;
}

/**
 * Query the current of a single channel of the PDP.
 *
 * @return The current of one of the PDP channels (channels 0-15) in Amperes
 */
double PowerDistributionPanel::GetCurrent(int channel) const {
  int32_t status = 0;

  if (!CheckPDPChannel(channel)) {
    std::stringstream buf;
    buf << "PDP Channel " << channel;
    WPI_LIB_FATAL_ERROR(ChannelIndexOutOfRange, buf.str());
  }

  double current = HAL_GetPDPChannelCurrent(m_handle, channel, &status);

  if (status) {
    WPI_LIB_FATAL_ERROR(Timeout, "");
  }

  return current;
}

/**
 * Query the total current of all monitored PDP channels (0-15).
 *
 * @return The the total current drawn from the PDP channels in Amperes
 */
double PowerDistributionPanel::GetTotalCurrent() const {
  int32_t status = 0;

  double current = HAL_GetPDPTotalCurrent(m_handle, &status);

  if (status) {
    WPI_LIB_FATAL_ERROR(Timeout, "");
  }

  return current;
}

/**
 * Query the total power drawn from the monitored PDP channels.
 *
 * @return The the total power drawn from the PDP channels in Watts
 */
double PowerDistributionPanel::GetTotalPower() const {
  int32_t status = 0;

  double power = HAL_GetPDPTotalPower(m_handle, &status);

  if (status) {
    WPI_LIB_FATAL_ERROR(Timeout, "");
  }

  return power;
}

/**
 * Query the total energy drawn from the monitored PDP channels.
 *
 * @return The the total energy drawn from the PDP channels in Joules
 */
double PowerDistributionPanel::GetTotalEnergy() const {
  int32_t status = 0;

  double energy = HAL_GetPDPTotalEnergy(m_handle, &status);

  if (status) {
    WPI_LIB_FATAL_ERROR(Timeout, "");
  }

  return energy;
}

/**
 * Reset the total energy drawn from the PDP.
 *
 * @see PowerDistributionPanel#GetTotalEnergy
 */
void PowerDistributionPanel::ResetTotalEnergy() {
  int32_t status = 0;

  HAL_ResetPDPTotalEnergy(m_handle, &status);

  if (status) {
    WPI_LIB_FATAL_ERROR(Timeout, "");
  }
}

/**
 * Remove all of the fault flags on the PDP.
 */
void PowerDistributionPanel::ClearStickyFaults() {
  int32_t status = 0;

  HAL_ClearPDPStickyFaults(m_handle, &status);

  if (status) {
    WPI_LIB_FATAL_ERROR(Timeout, "");
  }
}
