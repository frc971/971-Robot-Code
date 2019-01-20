/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>
#include <string>

#include "frc971/wpilib/ahal/PWM.h"

namespace frc {

/**
 * Standard hobby style servo.
 *
 * The range parameters default to the appropriate values for the Hitec HS-322HD
 * servo provided
 * in the FIRST Kit of Parts in 2008.
 */
class Servo : public PWM {
 public:
  explicit Servo(int channel);
  virtual ~Servo();

 private:
  static constexpr double kDefaultMaxServoPWM = 2.4;
  static constexpr double kDefaultMinServoPWM = .6;
};

}  // namespace frc
