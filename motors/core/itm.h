#ifndef MOTORS_CORE_ITM_H_
#define MOTORS_CORE_ITM_H_

#include "motors/core/reg_debug.h"

namespace frc971 {
namespace motors {

// Manages the ITM (Instrumentation and Trace Macrocell), along with the TPIU
// (Trace Port Interface Unit) and DWT (Data Watchpoint and Trace unit).
namespace itm {

void Initialize();

// Writes an 8-bit value to the specified stimulus port.
//
// This may be called without external synchronization simultaneously from any
// context (including all interrupts and other fault handlers).
inline void Write8(int port, uint8_t value) {
  const volatile uint32_t *const address = &ITM.STIM[port];
  uint32_t scratch;
  // This what ARM recommends in the Cortex-M3 TRM for concurrent usage by
  // interrupts etc. It's basically a compare-exchange from 0 to value without
  // any memory barriers, except actual C/C++ atomics are allergic to volatile.
  // See here for an example:
  // https://developer.arm.com/docs/ddi0337/e/system-debug/itm/summary-and-description-of-the-itm-registers
  __asm__ __volatile__(
      // Load the status (whether the FIFO is full).
      "1: ldrexb %[scratch], [%[address]]\n\t"
      // Check if it's reporting full or not.
      "cmp %[scratch], #0\n\t"
      "itt ne\n\t"
      // If it's not full, try storing our data.
      "strexbne %[scratch], %[value], [%[address]]\n\t"
      // If it's not full, check if our store succeeded.
      "cmpne %[scratch], #1\n\t"
      // If it's full or the store failed, try again.
      "beq 1b\n\t"
      : [scratch] "=&r"(scratch)
      : [address] "r"(address), [value] "r"(value)
      : "cc");
}

// See documentation for Write8.
inline void Write16(int port, uint16_t value) {
  const volatile uint32_t *const address = &ITM.STIM[port];
  uint32_t scratch;
  __asm__ __volatile__(
      // Load the status (whether the FIFO is full).
      "1: ldrexh %[scratch], [%[address]]\n\t"
      // Check if it's reporting full or not.
      "cmp %[scratch], #0\n\t"
      "itt ne\n\t"
      // If it's not full, try storing our data.
      "strexhne %[scratch], %[value], [%[address]]\n\t"
      // If it's not full, check if our store succeeded.
      "cmpne %[scratch], #1\n\t"
      // If it's full or the store failed, try again.
      "beq 1b\n\t"
      : [scratch] "=&r"(scratch)
      : [address] "r"(address), [value] "r"(value)
      : "cc");
}

// See documentation for Write8.
inline void Write32(int port, uint32_t value) {
  const volatile uint32_t *const address = &ITM.STIM[port];
  uint32_t scratch;
  __asm__ __volatile__(
      // Load the status (whether the FIFO is full).
      "1: ldrex %[scratch], [%[address]]\n\t"
      // Check if it's reporting full or not.
      "cmp %[scratch], #0\n\t"
      "itt ne\n\t"
      // If it's not full, try storing our data.
      "strexne %[scratch], %[value], [%[address]]\n\t"
      // If it's not full, check if our store succeeded.
      "cmpne %[scratch], #1\n\t"
      // If it's full or the store failed, try again.
      "beq 1b\n\t"
      : [scratch] "=&r"(scratch)
      : [address] "r"(address), [value] "r"(value)
      : "cc");
}

}  // namespace itm
}  // namespace motors
}  // namespace frc971

#endif  // MOTORS_CORE_ITM_H_
