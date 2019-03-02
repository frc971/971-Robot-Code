/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>

#include "frc971/wpilib/ahal/AnalogTriggerType.h"
#include "frc971/wpilib/ahal/SensorBase.h"
#include "hal/Interrupts.h"
#include "hal/cpp/fpga_clock.h"

namespace frc {

class InterruptableSensorBase {
 public:
  enum WaitResult {
    kTimeout = 0x0,
    kRisingEdge = 0x1,
    kFallingEdge = 0x100,
    kBoth = 0x101,
  };

  InterruptableSensorBase();
  virtual ~InterruptableSensorBase() = default;

  virtual HAL_Handle GetPortHandleForRouting() const = 0;
  virtual AnalogTriggerType GetAnalogTriggerTypeForRouting() const = 0;

  // Requests interrupts in synchronous mode. This means you should call
  // WaitForInterrupt to receive interrupts.
  virtual void RequestInterrupts();

  // Prevents any more interrupts from occuring.
  virtual void CancelInterrupts();

  // Waits for an interrupt or timeout to occur.
  //
  // Must be synchronized with all other operations on the input.
  //
  // timeout is in seconds.
  virtual WaitResult WaitForInterrupt(double timeout,
                                      bool ignorePrevious = true);

  // Enables interrupts to occur based on the current configuration.
  virtual void EnableInterrupts();

  // Returns the timestamp for the most recent rising interrupt.
  virtual hal::fpga_clock::time_point ReadRisingTimestamp();
  // Returns the timestamp for the most recent falling interrupt.
  virtual hal::fpga_clock::time_point ReadFallingTimestamp();

  // Configures which edges to interrupt on.
  //
  // The default is on rising edges only.
  virtual void SetUpSourceEdge(bool risingEdge, bool fallingEdge);

 protected:
  HAL_InterruptHandle m_interrupt = HAL_kInvalidHandle;
  void AllocateInterrupts(bool watcher);
};

}  // namespace frc
