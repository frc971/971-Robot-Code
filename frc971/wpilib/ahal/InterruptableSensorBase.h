/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>

#include "HAL/Interrupts.h"
#include "frc971/wpilib/ahal/AnalogTriggerType.h"
#include "frc971/wpilib/ahal/SensorBase.h"

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
  virtual void RequestInterrupts(
      HAL_InterruptHandlerFunction handler,
      void *param);                  ///< Asynchronous handler version.
  virtual void RequestInterrupts();  ///< Synchronous Wait version.
  virtual void
  CancelInterrupts();  ///< Free up the underlying chipobject functions.
  virtual WaitResult WaitForInterrupt(
      double timeout,
      bool ignorePrevious = true);  ///< Synchronous version.
  virtual void
  EnableInterrupts();  ///< Enable interrupts - after finishing setup.
  virtual void DisableInterrupts();       ///< Disable, but don't deallocate.
  virtual double ReadRisingTimestamp();   ///< Return the timestamp for the
                                          /// rising interrupt that occurred.
  virtual double ReadFallingTimestamp();  ///< Return the timestamp for the
                                          /// falling interrupt that occurred.
  virtual void SetUpSourceEdge(bool risingEdge, bool fallingEdge);

 protected:
  HAL_InterruptHandle m_interrupt = HAL_kInvalidHandle;
  void AllocateInterrupts(bool watcher);
};

}  // namespace frc
