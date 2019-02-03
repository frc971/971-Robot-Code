/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>
#include <string>

#include "hal/Types.h"
#include "frc971/wpilib/ahal/SensorBase.h"

namespace frc {

/**
 * Analog input class.
 *
 * Connected to each analog channel is an averaging and oversampling engine.
 * This engine accumulates the specified ( by SetAverageBits() and
 * SetOversampleBits() ) number of samples before returning a new value.  This
 * is not a sliding window average.  The only difference between the oversampled
 * samples and the averaged samples is that the oversampled samples are simply
 * accumulated effectively increasing the resolution, while the averaged samples
 * are divided by the number of samples to retain the resolution, but get more
 * stable values.
 */
class AnalogInput {
  friend class AnalogTrigger;
  friend class AnalogGyro;

 public:
  explicit AnalogInput(int channel);
  virtual ~AnalogInput();

  int GetValue() const;
  int GetAverageValue() const;

  double GetVoltage() const;
  double GetAverageVoltage() const;

  int GetChannel() const;

  void SetAverageBits(int bits);
  int GetAverageBits() const;
  void SetOversampleBits(int bits);
  int GetOversampleBits() const;

  int GetLSBWeight() const;
  int GetOffset() const;

  static void SetSampleRate(double samplesPerSecond);
  static double GetSampleRate();

 private:
  int m_channel;
  HAL_AnalogInputHandle m_port;
};

}  // namespace frc
