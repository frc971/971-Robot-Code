/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/SPI.h"
#include "frc971/wpilib/ahal/SPI.h"

#include <cstring>

#include "hal/HAL.h"
#include "wpi/SmallVector.h"

using namespace frc;

#define HAL_FATAL_WITH_STATUS(status)

SPI::SPI(Port SPIport) {
#ifdef WPILIB2017
  m_port = SPIport;
#else
  m_port = static_cast<HAL_SPIPort>(SPIport);
#endif
  int32_t status = 0;
  HAL_InitializeSPI(m_port, &status);
  HAL_FATAL_WITH_STATUS(status);

  static int instances = 0;
  instances++;
  HAL_Report(HALUsageReporting::kResourceType_SPI, instances);
}

SPI::~SPI() { HAL_CloseSPI(m_port); }

void SPI::SetClockRate(double hz) { HAL_SetSPISpeed(m_port, hz); }

void SPI::SetMSBFirst() {
  m_msbFirst = true;
  HAL_SetSPIOpts(m_port, m_msbFirst, m_sampleOnTrailing, m_clk_idle_high);
}

void SPI::SetLSBFirst() {
  m_msbFirst = false;
  HAL_SetSPIOpts(m_port, m_msbFirst, m_sampleOnTrailing, m_clk_idle_high);
}

void SPI::SetSampleDataOnFalling() {
  m_sampleOnTrailing = true;
  HAL_SetSPIOpts(m_port, m_msbFirst, m_sampleOnTrailing, m_clk_idle_high);
}

void SPI::SetSampleDataOnRising() {
  m_sampleOnTrailing = false;
  HAL_SetSPIOpts(m_port, m_msbFirst, m_sampleOnTrailing, m_clk_idle_high);
}

void SPI::SetClockActiveLow() {
  m_clk_idle_high = true;
  HAL_SetSPIOpts(m_port, m_msbFirst, m_sampleOnTrailing, m_clk_idle_high);
}

void SPI::SetClockActiveHigh() {
  m_clk_idle_high = false;
  HAL_SetSPIOpts(m_port, m_msbFirst, m_sampleOnTrailing, m_clk_idle_high);
}

void SPI::SetChipSelectActiveHigh() {
  int32_t status = 0;
  HAL_SetSPIChipSelectActiveHigh(m_port, &status);
  HAL_FATAL_WITH_STATUS(status);
}

void SPI::SetChipSelectActiveLow() {
  int32_t status = 0;
  HAL_SetSPIChipSelectActiveLow(m_port, &status);
  HAL_FATAL_WITH_STATUS(status);
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
