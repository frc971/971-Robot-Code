#ifndef CAPE_UTIL_H_
#define CAPE_UTIL_H_

#include <STM32F2XX.h>

#define ALIAS_WEAK(f) __attribute__ ((weak, alias (#f)))

// MSG has to be separated_with_spaces.
#define STATIC_ASSERT(COND,MSG) typedef char static_assertion_##MSG[(!!(COND))*2-1]

// Prevents the compiler from reordering memory operations around this.
static inline void compiler_memory_barrier(void) {
  __asm__ __volatile__("" ::: "memory");
}

// Sets number_of_bits (shifted left shift number of slots) to value in
// variable.
// This means that the total shift is number_bits*shift.
#define SET_BITS(variable, number_bits, value, shift) do { \
  variable = (((variable) & \
               ~(((1 << (number_bits)) - 1) << (shift * (number_bits)))) | \
              ((value) << (shift * (number_bits)))); \
} while (0);

// A convenient way to set up a GPIO pin for some alternate function without
// missing part or messing up which bits need setting to what.
// pin is the 0-indexed pin number.
// afr is 0-0xF for the various alternate functions.
static inline void gpio_setup_alt(GPIO_TypeDef *port, int pin, int afr) {
  SET_BITS(port->MODER, 2, 2 /* alternate function */, pin);
  if (pin < 8) {
    SET_BITS(port->AFR[0], 4, afr, pin);
  } else {
    SET_BITS(port->AFR[1], 4, afr, (pin - 8));
  }
}

#endif  // CAPE_UTIL_H_
