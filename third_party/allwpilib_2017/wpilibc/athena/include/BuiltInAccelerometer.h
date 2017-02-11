/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2014-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>
#include <string>

#include "SensorBase.h"

namespace frc {

/**
 * Built-in accelerometer.
 *
 * This class allows access to the roboRIO's internal accelerometer.
 */
class BuiltInAccelerometer : public SensorBase {
 public:
  enum Range { kRange_2G = 0, kRange_4G = 1, kRange_8G = 2 };

  explicit BuiltInAccelerometer(Range range = kRange_8G);
  virtual ~BuiltInAccelerometer() = default;

  void SetRange(Range range);
  double GetX();
  double GetY();
  double GetZ();
};

}  // namespace frc
