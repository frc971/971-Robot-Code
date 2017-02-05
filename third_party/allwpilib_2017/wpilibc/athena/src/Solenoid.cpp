/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "HAL/Solenoid.h"
#include "Solenoid.h"

#include <sstream>

#include "HAL/HAL.h"
#include "HAL/Ports.h"
#include "WPIErrors.h"

using namespace frc;

/**
 * Constructor using the default PCM ID (0).
 *
 * @param channel The channel on the PCM to control (0..7).
 */
Solenoid::Solenoid(int channel)
    : Solenoid(GetDefaultSolenoidModule(), channel) {}

/**
 * Constructor.
 *
 * @param moduleNumber The CAN ID of the PCM the solenoid is attached to
 * @param channel      The channel on the PCM to control (0..7).
 */
Solenoid::Solenoid(int moduleNumber, int channel)
    : SolenoidBase(moduleNumber), m_channel(channel) {
  std::stringstream buf;
  if (!CheckSolenoidModule(m_moduleNumber)) {
    buf << "Solenoid Module " << m_moduleNumber;
    wpi_setWPIErrorWithContext(ModuleIndexOutOfRange, buf.str());
    return;
  }
  if (!CheckSolenoidChannel(m_channel)) {
    buf << "Solenoid Module " << m_channel;
    wpi_setWPIErrorWithContext(ChannelIndexOutOfRange, buf.str());
    return;
  }

  int32_t status = 0;
  m_solenoidHandle = HAL_InitializeSolenoidPort(
      HAL_GetPortWithModule(moduleNumber, channel), &status);
  if (status != 0) {
    wpi_setErrorWithContextRange(status, 0, HAL_GetNumSolenoidChannels(),
                                 channel, HAL_GetErrorMessage(status));
    m_solenoidHandle = HAL_kInvalidHandle;
    return;
  }

  HAL_Report(HALUsageReporting::kResourceType_Solenoid, m_channel,
             m_moduleNumber);
}

/**
 * Destructor.
 */
Solenoid::~Solenoid() {
  HAL_FreeSolenoidPort(m_solenoidHandle);
}

/**
 * Set the value of a solenoid.
 *
 * @param on Turn the solenoid output off or on.
 */
void Solenoid::Set(bool on) {
  if (StatusIsFatal()) return;
  int32_t status = 0;
  HAL_SetSolenoid(m_solenoidHandle, on, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
}

/**
 * Read the current value of the solenoid.
 *
 * @return The current value of the solenoid.
 */
bool Solenoid::Get() const {
  if (StatusIsFatal()) return false;
  int32_t status = 0;
  bool value = HAL_GetSolenoid(m_solenoidHandle, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  return value;
}

/**
 * Check if solenoid is blacklisted.
 *
 * If a solenoid is shorted, it is added to the blacklist and
 * disabled until power cycle, or until faults are cleared.
 *
 * @see ClearAllPCMStickyFaults()
 *
 * @return If solenoid is disabled due to short.
 */
bool Solenoid::IsBlackListed() const {
  int value = GetPCMSolenoidBlackList(m_moduleNumber) & (1 << m_channel);
  return (value != 0);
}
