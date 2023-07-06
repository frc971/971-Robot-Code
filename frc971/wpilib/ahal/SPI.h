/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <hal/SPITypes.h>

#include <cstdint>
#include <memory>
#include <span>

#include "absl/types/span.h"
#include <wpi/deprecated.h>

namespace frc {

class DigitalSource;

/**
 * SPI bus interface class.
 *
 * This class is intended to be used by sensor (and other SPI device) drivers.
 * It probably should not be used directly.
 *
 */
class SPI final {
 public:
  enum Port { kOnboardCS0 = 0, kOnboardCS1, kOnboardCS2, kOnboardCS3, kMXP };
  enum Mode {
    kMode0 = HAL_SPI_kMode0,
    kMode1 = HAL_SPI_kMode1,
    kMode2 = HAL_SPI_kMode2,
    kMode3 = HAL_SPI_kMode3
  };

  /**
   * Constructor
   *
   * @param port the physical SPI port
   */
  explicit SPI(Port port);

  ~SPI();

  SPI(SPI &&) = default;
  SPI &operator=(SPI &&) = default;

  /**
   * Configure the rate of the generated clock signal.
   *
   * The default value is 500,000Hz.
   * The maximum value is 4,000,000Hz.
   *
   * @param hz The clock rate in Hertz.
   */
  void SetClockRate(int hz);

  /**
   * Configure the order that bits are sent and received on the wire
   * to be most significant bit first.
   *
   * @deprecated Does not work, will be removed.
   */
  WPI_DEPRECATED("Not supported by roboRIO.")
  void SetMSBFirst();

  /**
   * Configure the order that bits are sent and received on the wire
   * to be least significant bit first.
   *
   * @deprecated Does not work, will be removed.
   */
  WPI_DEPRECATED("Not supported by roboRIO.")
  void SetLSBFirst();

  /**
   * Configure that the data is stable on the leading edge and the data
   * changes on the trailing edge.
   *
   * @deprecated Use SetMode() instead.
   */
  WPI_DEPRECATED("Use SetMode() instead")
  void SetSampleDataOnLeadingEdge();

  /**
   * Configure that the data is stable on the trailing edge and the data
   * changes on the leading edge.
   *
   * @deprecated Use SetMode() instead.
   */
  WPI_DEPRECATED("Use SetMode() instead")
  void SetSampleDataOnTrailingEdge();

  /**
   * Configure the clock output line to be active low.
   * This is sometimes called clock polarity high or clock idle high.
   *
   * @deprecated Use SetMode() instead.
   */
  WPI_DEPRECATED("Use SetMode() instead")
  void SetClockActiveLow();

  /**
   * Configure the clock output line to be active high.
   * This is sometimes called clock polarity low or clock idle low.
   *
   * @deprecated Use SetMode() instead.
   */
  WPI_DEPRECATED("Use SetMode() instead")
  void SetClockActiveHigh();

  /**
   * Sets the mode for the SPI device.
   *
   * <p>Mode 0 is Clock idle low, data sampled on rising edge
   *
   * <p>Mode 1 is Clock idle low, data sampled on falling edge
   *
   * <p>Mode 2 is Clock idle high, data sampled on falling edge
   *
   * <p>Mode 3 is Clock idle high, data sampled on rising edge
   *
   * @param mode The mode to set.
   */
  void SetMode(Mode mode);

  /**
   * Configure the chip select line to be active high.
   */
  void SetChipSelectActiveHigh();

  /**
   * Configure the chip select line to be active low.
   */
  void SetChipSelectActiveLow();

  /**
   * Write data to the slave device.  Blocks until there is space in the
   * output FIFO.
   *
   * If not running in output only mode, also saves the data received
   * on the MISO input during the transfer into the receive FIFO.
   */
  virtual int Write(uint8_t *data, int size);

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
  virtual int Read(bool initiate, uint8_t *dataReceived, int size);

  /**
   * Perform a simultaneous read/write transaction with the device
   *
   * @param dataToSend   The data to be written out to the device
   * @param dataReceived Buffer to receive data from the device
   * @param size         The length of the transaction, in bytes
   */
  virtual int Transaction(uint8_t *dataToSend, uint8_t *dataReceived, int size);

  /**
   * Initialize automatic SPI transfer engine.
   *
   * Only a single engine is available, and use of it blocks use of all other
   * chip select usage on the same physical SPI port while it is running.
   *
   * @param bufferSize buffer size in bytes
   */
  void InitAuto(int bufferSize);

  /**
   * Frees the automatic SPI transfer engine.
   */
  void FreeAuto();

  /**
   * Set the data to be transmitted by the engine.
   *
   * Up to 16 bytes are configurable, and may be followed by up to 127 zero
   * bytes.
   *
   * @param dataToSend data to send (maximum 16 bytes)
   * @param zeroSize number of zeros to send after the data
   */
  void SetAutoTransmitData(absl::Span<const uint8_t> dataToSend, int zeroSize);

  /**
   * Start running the automatic SPI transfer engine at a periodic rate.
   *
   * InitAuto() and SetAutoTransmitData() must be called before calling this
   * function.
   *
   * @param period period between transfers, in seconds (us resolution)
   */
  void StartAutoRate(double period);

  /**
   * Start running the automatic SPI transfer engine when a trigger occurs.
   *
   * InitAuto() and SetAutoTransmitData() must be called before calling this
   * function.
   *
   * @param source digital source for the trigger (may be an analog trigger)
   * @param rising trigger on the rising edge
   * @param falling trigger on the falling edge
   */
  void StartAutoTrigger(DigitalSource &source, bool rising, bool falling);

  /**
   * Stop running the automatic SPI transfer engine.
   */
  void StopAuto();

  /**
   * Force the engine to make a single transfer.
   */
  void ForceAutoRead();

  /**
   * Read data that has been transferred by the automatic SPI transfer engine.
   *
   * Transfers may be made a byte at a time, so it's necessary for the caller
   * to handle cases where an entire transfer has not been completed.
   *
   * Each received data sequence consists of a timestamp followed by the
   * received data bytes, one byte per word (in the least significant byte).
   * The length of each received data sequence is the same as the combined
   * size of the data and zeroSize set in SetAutoTransmitData().
   *
   * Blocks until numToRead words have been read or timeout expires.
   * May be called with numToRead=0 to retrieve how many words are available.
   *
   * @param buffer buffer where read words are stored
   * @param numToRead number of words to read
   * @param timeout timeout in seconds (ms resolution)
   * @return Number of words remaining to be read
   */
  int ReadAutoReceivedData(uint32_t *buffer, int numToRead, double timeout);

  /**
   * Get the number of bytes dropped by the automatic SPI transfer engine due
   * to the receive buffer being full.
   *
   * @return Number of bytes dropped
   */
  int GetAutoDroppedCount();

  /**
   * Configure the Auto SPI Stall time between reads.
   *
   * @param port The number of the port to use. 0-3 for Onboard CS0-CS2, 4 for
   * MXP.
   * @param csToSclkTicks the number of ticks to wait before asserting the cs
   * pin
   * @param stallTicks the number of ticks to stall for
   * @param pow2BytesPerRead the number of bytes to read before stalling
   */
  void ConfigureAutoStall(int csToSclkTicks, int stallTicks,
                          int pow2BytesPerRead);

 protected:
  hal::SPIPort m_port;
  HAL_SPIMode m_mode = HAL_SPIMode::HAL_SPI_kMode0;

 private:
  void Init();
};

}  // namespace frc
