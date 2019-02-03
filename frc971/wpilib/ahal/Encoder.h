/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>
#include <string>

#include "hal/Encoder.h"

namespace frc {

class DigitalSource;
class DigitalGlitchFilter;

/**
 * Class to read quad encoders.
 * Quadrature encoders are devices that count shaft rotation and can sense
 * direction. The output of the QuadEncoder class is an integer that can count
 * either up or down, and can go negative for reverse direction counting. When
 * creating QuadEncoders, a direction is supplied that changes the sense of the
 * output to make code more readable if the encoder is mounted such that forward
 * movement generates negative values. Quadrature encoders have two digital
 * outputs, an A Channel and a B Channel that are out of phase with each other
 * to allow the FPGA to do direction sensing.
 *
 * All encoders will immediately start counting - Reset() them if you need them
 * to be zeroed before use.
 */
class Encoder {
 public:
  enum EncodingType { k1X, k2X, k4X };

  Encoder(int aChannel, int bChannel, bool reverseDirection = false,
          EncodingType encodingType = k4X);
  virtual ~Encoder();

  int GetEncodingScale() const;

  int GetRaw() const;

  double GetPeriod() const;
  void SetMaxPeriod(double maxPeriod);

  int GetFPGAIndex() const;

 private:
  void InitEncoder(bool reverseDirection, EncodingType encodingType);

  double DecodingScaleFactor() const;

  std::shared_ptr<DigitalSource> m_aSource;  // the A phase of the quad encoder
  std::shared_ptr<DigitalSource> m_bSource;  // the B phase of the quad encoder
  std::unique_ptr<DigitalSource> m_indexSource = nullptr;
  HAL_EncoderHandle m_encoder = HAL_kInvalidHandle;

  friend class DigitalGlitchFilter;
};

}  // namespace frc
