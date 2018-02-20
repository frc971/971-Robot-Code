/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc971/wpilib/ahal/Servo.h"

using namespace frc;

constexpr double Servo::kDefaultMaxServoPWM;
constexpr double Servo::kDefaultMinServoPWM;

/**
 * @param channel The PWM channel to which the servo is attached. 0-9 are
 *                on-board, 10-19 are on the MXP port
 */
Servo::Servo(int channel) : PWM(channel) {
  // Set minimum and maximum PWM values supported by the servo
  SetBounds(kDefaultMaxServoPWM, 0.0, 0.0, 0.0, kDefaultMinServoPWM);

  // Assign defaults for period multiplier for the servo PWM control signal
  SetPeriodMultiplier(kPeriodMultiplier_4X);
}

Servo::~Servo() {}
