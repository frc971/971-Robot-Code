/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2016. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "MotorSafety.h"
#include "PWM.h"
#include "MotorSafetyHelper.h"
#include <memory>
#include <sstream>

/**
 * A safe version of the PWM class.
 * It is safe because it implements the MotorSafety interface that provides
 * timeouts
 * in the event that the motor value is not updated before the expiration time.
 * This delegates the actual work to a MotorSafetyHelper object that is used for
 * all
 * objects that implement MotorSafety.
 */
class SafePWM : public PWM
#if FULL_WPILIB
                ,
                public MotorSafety
#endif
                {
 public:
  explicit SafePWM(uint32_t channel);
  virtual ~SafePWM() = default;

#if FULL_WPILIB
  void SetExpiration(float timeout);
  float GetExpiration() const;
  bool IsAlive() const;
  void StopMotor();
  bool IsSafetyEnabled() const;
  void SetSafetyEnabled(bool enabled);
  void GetDescription(std::ostringstream& desc) const;
#endif

  virtual void SetSpeed(float speed);

 private:
#if FULL_WPILIB
  std::unique_ptr<MotorSafetyHelper> m_safetyHelper;
#endif
};
