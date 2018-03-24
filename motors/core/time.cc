#include "motors/core/time.h"

#include "motors/core/kinetis.h"

// The systick interrupt increments this every 1ms.
volatile uint32_t systick_millis_count = 0;

namespace {

template<int kMultiplier>
uint32_t do_time(void) {
  __disable_irq();
  uint32_t current = SYST_CVR;
  uint32_t count = systick_millis_count;
  const uint32_t istatus = SCB_ICSR;
  __enable_irq();
  // If the interrupt is pending and the timer has already wrapped from 0 back
  // up to its max, then add another ms.
  if ((istatus & SCB_ICSR_PENDSTSET) && current > 50) count++;
  current = ((F_CPU / 1000) - 1) - current;
  return count * (1000 * kMultiplier) +
         current * kMultiplier / (F_CPU / 1000000);
}

}  // namespace

uint32_t nanos(void) { return do_time<1000>(); }

uint32_t micros(void) { return do_time<1>(); }

uint32_t millis(void) { return systick_millis_count; }

void delay(uint32_t ms) { delay_from(micros(), ms); }

uint32_t delay_from(uint32_t start, uint32_t ms) {
  if (ms > 0) {
    while (1) {
      while ((uint32_t)(micros() - start) >= 1000u) {
        ms--;
        start += 1000;
        if (ms == 0) return start;
      }
    }
  }
  return start;
}
