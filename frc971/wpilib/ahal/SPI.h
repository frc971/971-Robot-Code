/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "hal/SPI.h"

namespace frc {

class DigitalOutput;
class DigitalInput;

/**
 * SPI bus interface class.
 *
 * This class is intended to be used by sensor (and other SPI device) drivers.
 * It probably should not be used directly.
 *
 */
class SPI {
 public:
  enum Port : int32_t {
    kOnboardCS0 = 0,
    kOnboardCS1,
    kOnboardCS2,
    kOnboardCS3,
    kMXP
  };
  explicit SPI(Port SPIport);
  virtual ~SPI();

  SPI(const SPI &) = delete;
  SPI &operator=(const SPI &) = delete;

  // Configure the rate of the generated clock signal.
  //
  // The claimed default value is 500,000Hz, and the claimed maximum value is
  // 4,000,000Hz.
  //
  // This appears to have a very inflexible clocking setup. You can get 0.781MHz
  // or 1.563MHz, but nothing in between. At least it rounds down the requested
  // value like it should... 0.781MHz also appears to be the minimum.
  void SetClockRate(double hz);

  // Configure the order that bits are sent and received on the wire
  // to be most significant bit first.
  void SetMSBFirst();
  // Configure the order that bits are sent and received on the wire
  // to be least significant bit first.
  void SetLSBFirst();

  // Configure that the data is stable on the falling edge and the data
  // changes on the rising edge.
  void SetSampleDataOnFalling();
  // Configure that the data is stable on the rising edge and the data
  // changes on the falling edge.
  void SetSampleDataOnRising();

  // Configure the clock output line to be active low.
  // This is sometimes called clock polarity high or clock idle high.
  void SetClockActiveLow();
  // Configure the clock output line to be active high.
  // This is sometimes called clock polarity low or clock idle low.
  void SetClockActiveHigh();

  // Configure the chip select line to be active high.
  void SetChipSelectActiveHigh();
  // Configure the chip select line to be active low.
  void SetChipSelectActiveLow();

  // Write data to the slave device.  Blocks until there is space in the
  // output FIFO.
  //
  // If not running in output only mode, also saves the data received
  // on the MISO input during the transfer into the receive FIFO.
  int Write(uint8_t *data, int size);
  // Read a word from the receive FIFO.
  //
  // Waits for the current transfer to complete if the receive FIFO is empty.
  //
  // If the receive FIFO is empty, there is no active transfer, and initiate
  // is false, errors.
  //
  // @param initiate If true, this function pushes "0" into the transmit buffer
  //                 and initiates a transfer. If false, this function assumes
  //                 that data is already in the receive FIFO from a previous
  //                 write.
  int Read(bool initiate, uint8_t *dataReceived, int size);
  // Perform a simultaneous read/write transaction with the device
  //
  // @param dataToSend   The data to be written out to the device
  // @param dataReceived Buffer to receive data from the device
  // @param size         The length of the transaction, in bytes
  int Transaction(uint8_t *dataToSend, uint8_t *dataReceived, int size);

 protected:
#ifdef WPILIB2017
  int m_port;
#else
  HAL_SPIPort m_port;
#endif
  bool m_msbFirst = false;          // default little-endian
  bool m_sampleOnTrailing = false;  // default data updated on falling edge
  bool m_clk_idle_high = false;     // default clock active high

 private:
  void Init();
};

}  // namespace frc
