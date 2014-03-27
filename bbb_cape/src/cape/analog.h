#ifndef CAPE_ANALOG_H_
#define CAPE_ANALOG_H_

#include <stdint.h>

#include <STM32F2XX.h>

// Starts up constantly reading analog values and storing them in an array to
// be retrieved by analog_get.
void analog_init(void);

static inline uint16_t analog_get(int num) {
  if (num < 0 || num > 7) return 0xFFFF;

  extern uint16_t analog_readings[8] __attribute__((aligned(8)));
  return analog_readings[num];
}

// Returns the number of errors since last called.
// Must be called from something with priority equal to or lower than our
// timer's IRQ.
int analog_get_errors(void);

#endif  // CAPE_ANALOG_H_
