#ifndef CAPE_DIGITAL_H_
#define CAPE_DIGITAL_H_

#include <STM32F2XX.h>

void digital_init(void);

// For all of the digital functions, a high voltage level on the input reads as
// 1 (and a low to high transition is a positive edge).

static inline int digital_read(int num) {
  switch (num) {
    case 0:
      return !(GPIOC->IDR & (1 << 4));
    case 1:
      return !(GPIOC->IDR & (1 << 5));
    case 2:
      return !(GPIOC->IDR & (1 << 13));
    case 3:
      return !(GPIOC->IDR & (1 << 14));
    case 4:
      return !(GPIOC->IDR & (1 << 15));
    case 5:
      return !(GPIOB->IDR & (1 << 10));
    case 6:
      return !(GPIOB->IDR & (1 << 9));
    case 7:
      return !(GPIOB->IDR & (1 << 8));
    case 8:
      return !(GPIOA->IDR & (1 << 12));
    case 9:
      return !(GPIOA->IDR & (1 << 11));
    case 10:
      return !(GPIOA->IDR & (1 << 7));
    case 11:
      return !(GPIOB->IDR & (1 << 2));
    default:
      return 0;
  }
}

// A helper function for implementing digital_capture_{disable,enable}.
static inline enum IRQn digital_capture_getirqn(int num) {
  switch (num) {
    case 0:
      return EXTI4_IRQn;
    case 1:
      return EXTI9_5_IRQn;
    case 2:
      return EXTI15_10_IRQn;
    case 3:
      return EXTI15_10_IRQn;
    case 4:
      return EXTI15_10_IRQn;
    case 5:
      return EXTI15_10_IRQn;
    case 6:
      return EXTI9_5_IRQn;
    case 7:
      return EXTI9_5_IRQn;
    case 8:
      return EXTI15_10_IRQn;
    case 9:
      return EXTI15_10_IRQn;
    case 10:
      return EXTI9_5_IRQn;
    case 11:
      return EXTI2_IRQn;
    default:
      __builtin_trap();
  }
}

// May disable other capture inputs too.
static inline void digital_capture_disable(int num) {
  NVIC_DisableIRQ(digital_capture_getirqn(num));
}

// May enable other capture inputs too.
static inline void digital_capture_enable(int num) {
  NVIC_EnableIRQ(digital_capture_getirqn(num));
}

// These are the functions for handling edges on the inputs. They have
// default (weak symbol) implementations that do nothing.
//void digital_capture_0P(void);
//void digital_capture_0N(void);
//void digital_capture_1P(void);
//void digital_capture_1N(void);
//...

#endif  // CAPE_DIGITAL_H_
