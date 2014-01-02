#ifndef CAPE_UTIL_H_
#define CAPE_UTIL_H_

#include <stdint.h>

#include <STM32F2XX.h>

#define ALIAS_WEAK(f) __attribute__ ((weak, alias (#f)))

// MSG has to be separated_with_spaces.
#define STATIC_ASSERT(COND,MSG) typedef char static_assertion_##MSG[(!!(COND))*2-1]

// Prevents the compiler from reordering memory operations around this.
static inline void compiler_memory_barrier(void) {
  __asm__ __volatile__("" ::: "memory");
}

// Count leading zeros.
// Returns 0 if bit 31 is set etc.
__attribute__((always_inline)) static __INLINE uint32_t __clz(uint32_t value) {
  uint32_t result;
  __asm__("clz %0, %1" : "=r" (result) : "r" (value));
  return result;
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

// A convenient way to set up a GPIO pin for output (push-pull) without missing
// part or messing up which bits need setting to what.
// speed is 0 (slow) to 3 (fast)
static inline void gpio_setup_out(GPIO_TypeDef *port, int pin, int speed) {
  SET_BITS(port->MODER, 2, 1 /* output */, pin);
  SET_BITS(port->OSPEEDR, 2, speed, pin);
}

static inline void gpio_setup_in(GPIO_TypeDef *port, int pin) {
  SET_BITS(port->MODER, 2, 0 /* input */, pin);
}

// exti is which EXTI line to set
// port is 0 for A, 1 for B, etc
static inline void EXTI_set(int exti, int port) {
  SET_BITS(SYSCFG->EXTICR[exti / 4], 4, port, exti % 4);
}

static inline void gpio_on(GPIO_TypeDef *port, int pin) {
  port->BSRRL = 1 << pin;
}

static inline void gpio_off(GPIO_TypeDef *port, int pin) {
  port->BSRRH = 1 << pin;
}

void led_write(uint32_t value, int bits);

#endif  // CAPE_UTIL_H_
