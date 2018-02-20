/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "HAL/SPI.h"
#include "frc971/wpilib/ahal/SPI.h"

#include <cstring>

#include "HAL/HAL.h"
#include "llvm/SmallVector.h"

using namespace frc;

#define HAL_FATAL_WITH_STATUS(status)

/**
 * Constructor
 *
 * @param SPIport the physical SPI port
 */
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

/**
 * Destructor.
 */
SPI::~SPI() { HAL_CloseSPI(m_port); }

/**
 * Configure the rate of the generated clock signal.
 *
 * The default value is 500,000Hz.
 * The maximum value is 4,000,000Hz.
 *
 * @param hz The clock rate in Hertz.
 */
void SPI::SetClockRate(double hz) { HAL_SetSPISpeed(m_port, hz); }

/**
 * Configure the order that bits are sent and received on the wire
 * to be most significant bit first.
 */
void SPI::SetMSBFirst() {
  m_msbFirst = true;
  HAL_SetSPIOpts(m_port, m_msbFirst, m_sampleOnTrailing, m_clk_idle_high);
}

/**
 * Configure the order that bits are sent and received on the wire
 * to be least significant bit first.
 */
void SPI::SetLSBFirst() {
  m_msbFirst = false;
  HAL_SetSPIOpts(m_port, m_msbFirst, m_sampleOnTrailing, m_clk_idle_high);
}

/**
 * Configure that the data is stable on the falling edge and the data
 * changes on the rising edge.
 */
void SPI::SetSampleDataOnFalling() {
  m_sampleOnTrailing = true;
  HAL_SetSPIOpts(m_port, m_msbFirst, m_sampleOnTrailing, m_clk_idle_high);
}

/**
 * Configure that the data is stable on the rising edge and the data
 * changes on the falling edge.
 */
void SPI::SetSampleDataOnRising() {
  m_sampleOnTrailing = false;
  HAL_SetSPIOpts(m_port, m_msbFirst, m_sampleOnTrailing, m_clk_idle_high);
}

/**
 * Configure the clock output line to be active low.
 * This is sometimes called clock polarity high or clock idle high.
 */
void SPI::SetClockActiveLow() {
  m_clk_idle_high = true;
  HAL_SetSPIOpts(m_port, m_msbFirst, m_sampleOnTrailing, m_clk_idle_high);
}

/**
 * Configure the clock output line to be active high.
 * This is sometimes called clock polarity low or clock idle low.
 */
void SPI::SetClockActiveHigh() {
  m_clk_idle_high = false;
  HAL_SetSPIOpts(m_port, m_msbFirst, m_sampleOnTrailing, m_clk_idle_high);
}

/**
 * Configure the chip select line to be active high.
 */
void SPI::SetChipSelectActiveHigh() {
  int32_t status = 0;
  HAL_SetSPIChipSelectActiveHigh(m_port, &status);
  HAL_FATAL_WITH_STATUS(status);
}

/**
 * Configure the chip select line to be active low.
 */
void SPI::SetChipSelectActiveLow() {
  int32_t status = 0;
  HAL_SetSPIChipSelectActiveLow(m_port, &status);
  HAL_FATAL_WITH_STATUS(status);
}

/**
 * Write data to the slave device.  Blocks until there is space in the
 * output FIFO.
 *
 * If not running in output only mode, also saves the data received
 * on the MISO input during the transfer into the receive FIFO.
 */
int SPI::Write(uint8_t *data, int size) {
  int retVal = 0;
  retVal = HAL_WriteSPI(m_port, data, size);
  return retVal;
}

/**
 * Read a word from the receive FIFO.
 *
 * Waits for the current transfer to complete if the receive FIFO is empty.
 *
 * If the receive FIFO is empty, there is no active transfer, and initiate
 * is false, errors.
 *
 * @param initiate If true, this function pushes "0" into the transmit buffer
 *                 and initiates a transfer. If false, this function assumes
 *                 that data is already in the receive FIFO from a previous
 *                 write.
 */
int SPI::Read(bool initiate, uint8_t *dataReceived, int size) {
  int retVal = 0;
  if (initiate) {
    llvm::SmallVector<uint8_t, 32> dataToSend;
    dataToSend.resize(size);
    retVal = HAL_TransactionSPI(m_port, dataToSend.data(), dataReceived, size);
  } else {
    retVal = HAL_ReadSPI(m_port, dataReceived, size);
  }
  return retVal;
}

/**
 * Perform a simultaneous read/write transaction with the device
 *
 * @param dataToSend   The data to be written out to the device
 * @param dataReceived Buffer to receive data from the device
 * @param size         The length of the transaction, in bytes
 */
int SPI::Transaction(uint8_t *dataToSend, uint8_t *dataReceived, int size) {
  int retVal = 0;
  retVal = HAL_TransactionSPI(m_port, dataToSend, dataReceived, size);
  return retVal;
}
