/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "HAL/SPI.h"

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

  void SetClockRate(double hz);

  void SetMSBFirst();
  void SetLSBFirst();

  void SetSampleDataOnFalling();
  void SetSampleDataOnRising();

  void SetClockActiveLow();
  void SetClockActiveHigh();

  void SetChipSelectActiveHigh();
  void SetChipSelectActiveLow();

  virtual int Write(uint8_t *data, int size);
  virtual int Read(bool initiate, uint8_t *dataReceived, int size);
  virtual int Transaction(uint8_t *dataToSend, uint8_t *dataReceived, int size);

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
