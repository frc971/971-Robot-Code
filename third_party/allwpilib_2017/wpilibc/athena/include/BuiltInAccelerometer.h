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
#include "interfaces/Accelerometer.h"

namespace frc {

/**
 * Built-in accelerometer.
 *
 * This class allows access to the roboRIO's internal accelerometer.
 */
class BuiltInAccelerometer : public Accelerometer,
                             public SensorBase {
 public:
  explicit BuiltInAccelerometer(Range range = kRange_8G);
  virtual ~BuiltInAccelerometer() = default;

  // Accelerometer interface
  void SetRange(Range range) override;
  double GetX() override;
  double GetY() override;
  double GetZ() override;
};

}  // namespace frc
