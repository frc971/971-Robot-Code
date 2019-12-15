/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>

#include <hal/Types.h>

#include "frc971/wpilib/ahal/AnalogTriggerOutput.h"

namespace frc {

class AnalogInput;
class DutyCycle;
class SendableBuilder;

class AnalogTrigger {
  friend class AnalogTriggerOutput;

 public:
  explicit AnalogTrigger(int channel);
  explicit AnalogTrigger(AnalogInput* channel);
  explicit AnalogTrigger(DutyCycle* dutyCycle);

  virtual ~AnalogTrigger();

  AnalogTrigger(AnalogTrigger&& rhs);
  AnalogTrigger& operator=(AnalogTrigger&& rhs);

  void SetLimitsVoltage(double lower, double upper);

  void SetLimitsDutyCycle(double lower, double upper);

  void SetLimitsRaw(int lower, int upper);

  void SetAveraged(bool useAveragedValue);

  void SetDutyCycle(bool useDutyCycle);

  void SetFiltered(bool useFilteredValue);

  int GetIndex() const;

  bool GetInWindow();

  bool GetTriggerState();

  std::shared_ptr<AnalogTriggerOutput> CreateOutput(
      AnalogTriggerType type) const;

 private:
  hal::Handle<HAL_AnalogTriggerHandle> m_trigger;
  AnalogInput* m_analogInput = nullptr;
  DutyCycle* m_dutyCycle = nullptr;
  bool m_ownsAnalog = false;
};

}  // namespace frc
