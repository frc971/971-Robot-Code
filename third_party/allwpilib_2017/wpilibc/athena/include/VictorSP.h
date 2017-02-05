/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "PWM.h"

namespace frc {

/**
 * Vex Robotics Victor SP Speed Controller
 */
class VictorSP : public PWM {
 public:
  explicit VictorSP(int channel);
  virtual ~VictorSP() = default;
};

}  // namespace frc
