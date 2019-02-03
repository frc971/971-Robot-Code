/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc971/wpilib/ahal/InterruptableSensorBase.h"

#include "hal/HAL.h"
#include "frc971/wpilib/ahal/Utility.h"
#include "frc971/wpilib/ahal/WPIErrors.h"

using namespace frc;

namespace {

// Converts a freestanding lower half to a 64 bit FPGA timestamp
//
// Note: This is making the assumption that the timestamp being converted is
// always in the past.  If you call this with a future timestamp, it probably
// will make it in the past.  If you wait over 70 minutes between capturing the
// bottom 32 bits of the timestamp and expanding it, you will be off by
// multiples of 1<<32 microseconds.
//
// @return The current time in microseconds according to the FPGA (since FPGA
// reset) as a 64 bit number.
uint64_t HAL_ExpandFPGATime(uint32_t unexpanded_lower, int32_t* status) {
  // Capture the current FPGA time.  This will give us the upper half of the
  // clock.
  uint64_t fpga_time = HAL_GetFPGATime(status);
  if (*status != 0) return 0;

  // Now, we need to detect the case where the lower bits rolled over after we
  // sampled.  In that case, the upper bits will be 1 bigger than they should
  // be.

  // Break it into lower and upper portions.
  uint32_t lower = fpga_time & ((uint64_t)0xffffffff);
  uint64_t upper = (fpga_time >> 32) & 0xffffffff;

  // The time was sampled *before* the current time, so roll it back.
  if (lower < unexpanded_lower) {
    --upper;
  }

  return (upper << 32) + static_cast<uint64_t>(unexpanded_lower);
}

}  // namespace

InterruptableSensorBase::InterruptableSensorBase() {}

void InterruptableSensorBase::RequestInterrupts() {
  if (StatusIsFatal()) return;

  wpi_assert(m_interrupt == HAL_kInvalidHandle);
  AllocateInterrupts(true);
  if (StatusIsFatal()) return;  // if allocate failed, out of interrupts

  int32_t status = 0;
  HAL_RequestInterrupts(
      m_interrupt, GetPortHandleForRouting(),
      static_cast<HAL_AnalogTriggerType>(GetAnalogTriggerTypeForRouting()),
      &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  SetUpSourceEdge(true, false);
}

void InterruptableSensorBase::AllocateInterrupts(bool watcher) {
  wpi_assert(m_interrupt == HAL_kInvalidHandle);
  // Expects the calling leaf class to allocate an interrupt index.
  int32_t status = 0;
  m_interrupt = HAL_InitializeInterrupts(watcher, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
}

void InterruptableSensorBase::CancelInterrupts() {
  if (StatusIsFatal()) return;
  wpi_assert(m_interrupt != HAL_kInvalidHandle);
  int32_t status = 0;
  HAL_CleanInterrupts(m_interrupt, &status);
  // ignore status, as an invalid handle just needs to be ignored.
  m_interrupt = HAL_kInvalidHandle;
}

InterruptableSensorBase::WaitResult InterruptableSensorBase::WaitForInterrupt(
    double timeout, bool ignorePrevious) {
  if (StatusIsFatal()) return InterruptableSensorBase::kTimeout;
  wpi_assert(m_interrupt != HAL_kInvalidHandle);
  int32_t status = 0;
  int result;

  result = HAL_WaitForInterrupt(m_interrupt, timeout, ignorePrevious, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));

  // Rising edge result is the interrupt bit set in the byte 0xFF
  // Falling edge result is the interrupt bit set in the byte 0xFF00
  // Set any bit set to be true for that edge, and AND the 2 results
  // together to match the existing enum for all interrupts
  int32_t rising = (result & 0xFF) ? 0x1 : 0x0;
  int32_t falling = ((result & 0xFF00) ? 0x0100 : 0x0);
  return static_cast<WaitResult>(falling | rising);
}

void InterruptableSensorBase::EnableInterrupts() {
  if (StatusIsFatal()) return;
  wpi_assert(m_interrupt != HAL_kInvalidHandle);
  int32_t status = 0;
  HAL_EnableInterrupts(m_interrupt, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
}

hal::fpga_clock::time_point InterruptableSensorBase::ReadRisingTimestamp() {
  if (StatusIsFatal()) return hal::fpga_clock::min_time;
  wpi_assert(m_interrupt != HAL_kInvalidHandle);
  int32_t status = 0;
  uint64_t timestamp = HAL_ReadInterruptRisingTimestamp(m_interrupt, &status);
  timestamp = HAL_ExpandFPGATime(timestamp, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  return hal::fpga_clock::time_point(hal::fpga_clock::duration(timestamp));
}

hal::fpga_clock::time_point InterruptableSensorBase::ReadFallingTimestamp() {
  if (StatusIsFatal()) return hal::fpga_clock::min_time;
  wpi_assert(m_interrupt != HAL_kInvalidHandle);
  int32_t status = 0;
  uint64_t timestamp = HAL_ReadInterruptFallingTimestamp(m_interrupt, &status);
  timestamp = HAL_ExpandFPGATime(timestamp, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  return hal::fpga_clock::time_point(hal::fpga_clock::duration(timestamp));
}

void InterruptableSensorBase::SetUpSourceEdge(bool risingEdge,
                                              bool fallingEdge) {
  if (StatusIsFatal()) return;
  if (m_interrupt == HAL_kInvalidHandle) {
    wpi_setWPIErrorWithContext(
        NullParameter,
        "You must call RequestInterrupts before SetUpSourceEdge");
    return;
  }
  if (m_interrupt != HAL_kInvalidHandle) {
    int32_t status = 0;
    HAL_SetInterruptUpSourceEdge(m_interrupt, risingEdge, fallingEdge, &status);
    wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  }
}
