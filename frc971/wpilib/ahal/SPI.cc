/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc971/wpilib/ahal/SPI.h"

#include <hal/SPI.h>

#include <cstring>
#include <utility>

#include "absl/types/span.h"
#include <wpi/SmallVector.h>
#include <wpi/mutex.h>

#include "frc971/wpilib/ahal/DigitalSource.h"
#include "frc971/wpilib/ahal/WPIErrors.h"

namespace frc {

SPI::SPI(Port port) : m_port(static_cast<HAL_SPIPort>(port)) {
  int32_t status = 0;
  HAL_InitializeSPI(m_port, &status);
  wpi_setHALError(status);
}

SPI::~SPI() { HAL_CloseSPI(m_port); }

void SPI::SetClockRate(int hz) { HAL_SetSPISpeed(m_port, hz); }

void SPI::SetMode(Mode mode) {
  m_mode = static_cast<HAL_SPIMode>(mode & 0x3);
  HAL_SetSPIMode(m_port, m_mode);
}

void SPI::SetSampleDataOnLeadingEdge() {
  int mode = m_mode;
  mode &= 2;
  m_mode = static_cast<HAL_SPIMode>(mode);
  HAL_SetSPIMode(m_port, m_mode);
}

void SPI::SetSampleDataOnTrailingEdge() {
  int mode = m_mode;
  mode |= 2;
  m_mode = static_cast<HAL_SPIMode>(mode);
  HAL_SetSPIMode(m_port, m_mode);
}

void SPI::SetClockActiveLow() {
  int mode = m_mode;
  mode |= 1;
  m_mode = static_cast<HAL_SPIMode>(mode);
  HAL_SetSPIMode(m_port, m_mode);
}

void SPI::SetClockActiveHigh() {
  int mode = m_mode;
  mode &= 1;
  m_mode = static_cast<HAL_SPIMode>(mode);
  HAL_SetSPIMode(m_port, m_mode);
}

void SPI::SetChipSelectActiveHigh() {
  int32_t status = 0;
  HAL_SetSPIChipSelectActiveHigh(m_port, &status);
  wpi_setHALError(status);
}

void SPI::SetChipSelectActiveLow() {
  int32_t status = 0;
  HAL_SetSPIChipSelectActiveLow(m_port, &status);
  wpi_setHALError(status);
}

int SPI::Write(uint8_t *data, int size) {
  int retVal = 0;
  retVal = HAL_WriteSPI(m_port, data, size);
  return retVal;
}

int SPI::Read(bool initiate, uint8_t *dataReceived, int size) {
  int retVal = 0;
  if (initiate) {
    wpi::SmallVector<uint8_t, 32> dataToSend;
    dataToSend.resize(size);
    retVal = HAL_TransactionSPI(m_port, dataToSend.data(), dataReceived, size);
  } else {
    retVal = HAL_ReadSPI(m_port, dataReceived, size);
  }
  return retVal;
}

int SPI::Transaction(uint8_t *dataToSend, uint8_t *dataReceived, int size) {
  int retVal = 0;
  retVal = HAL_TransactionSPI(m_port, dataToSend, dataReceived, size);
  return retVal;
}

void SPI::InitAuto(int bufferSize) {
  int32_t status = 0;
  HAL_InitSPIAuto(m_port, bufferSize, &status);
  wpi_setHALError(status);
}

void SPI::FreeAuto() {
  int32_t status = 0;
  HAL_FreeSPIAuto(m_port, &status);
  wpi_setHALError(status);
}

void SPI::SetAutoTransmitData(absl::Span<const uint8_t> dataToSend,
                              int zeroSize) {
  int32_t status = 0;
  HAL_SetSPIAutoTransmitData(m_port, dataToSend.data(), dataToSend.size(),
                             zeroSize, &status);
  wpi_setHALError(status);
}

void SPI::StartAutoRate(double period) {
  int32_t status = 0;
  HAL_StartSPIAutoRate(m_port, period, &status);
  wpi_setHALError(status);
}

void SPI::StartAutoTrigger(DigitalSource &source, bool rising, bool falling) {
  int32_t status = 0;
  HAL_StartSPIAutoTrigger(
      m_port, source.GetPortHandleForRouting(),
      (HAL_AnalogTriggerType)source.GetAnalogTriggerTypeForRouting(), rising,
      falling, &status);
  wpi_setHALError(status);
}

void SPI::StopAuto() {
  int32_t status = 0;
  HAL_StopSPIAuto(m_port, &status);
  wpi_setHALError(status);
}

void SPI::ForceAutoRead() {
  int32_t status = 0;
  HAL_ForceSPIAutoRead(m_port, &status);
  wpi_setHALError(status);
}

int SPI::ReadAutoReceivedData(uint32_t *buffer, int numToRead, double timeout) {
  int32_t status = 0;
  int32_t val =
      HAL_ReadSPIAutoReceivedData(m_port, buffer, numToRead, timeout, &status);
  wpi_setHALError(status);
  return val;
}

int SPI::GetAutoDroppedCount() {
  int32_t status = 0;
  int32_t val = HAL_GetSPIAutoDroppedCount(m_port, &status);
  wpi_setHALError(status);
  return val;
}

void SPI::ConfigureAutoStall(int csToSclkTicks, int stallTicks,
                             int pow2BytesPerRead) {
  int32_t status = 0;
  HAL_ConfigureSPIAutoStall(m_port, csToSclkTicks, stallTicks, pow2BytesPerRead,
                            &status);
  wpi_setHALError(status);
}

}  // namespace frc
