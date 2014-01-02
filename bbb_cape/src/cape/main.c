#include <STM32F2XX.h>

#include "cape/fill_packet.h"
#include "cape/led.h"

// The startup asm code defines this to the start of our exception vector table.
extern uint32_t _vectors;

void _start(void) {
  led_set(LED_ERR, 1);
  led_set(LED_HB, 0);
  // Change the vector table offset to use our vector table instead of the
  // bootloader's.
  SCB->VTOR = (uint32_t)&_vectors;
  // Data Memory Barrier to make sure it gets the updated vector table.
  __asm__ __volatile__("dmb");

  fill_packet_start();

  // Make it go right to sleep after handling all exceptions. This actually
  // decreses ISR latency a little bit because it doesn't have to stack the
  // registers for the first one.
  SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;

  // This seems like the perfect place to use WFI, but Brian on 2013-12-13
  // couldn't find anything verifying that WFI doesn't increase the latency for
  // the first interrupt handled, and we should never actually get here anyways,
  // so it doesn't matter.
  while (1) {}
}
