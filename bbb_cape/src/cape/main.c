#include <STM32F2XX.h>

#include "cape/bootloader_handoff.h"

// The startup asm code defines this to the start of our exception vector table.
extern uint32_t _vectors;

void _start(void) {
  // Change the vector table offset to use our vector table instead of the
  // bootloader's.
  SCB->VTOR = (uint32_t)&_vectors;
}
