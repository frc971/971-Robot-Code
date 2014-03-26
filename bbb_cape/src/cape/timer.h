#ifndef CAPE_TIMER_H_
#define CAPE_TIMER_H_

#include <STM32F2XX.h>

// inverted is 1 for timing low periods, 0 for high ones.
#define timer_declare(timer, irq, input, inverted, name)                   \
  static volatile uint32_t name##_length;                                  \
  void irq(void) {                                                         \
    timer->SR = ~TIM_SR_CC##input##IF;                                     \
    const uint32_t ccer = timer->CCER;                                     \
    timer->CCER = ccer ^ (TIM_CCER_CC##input##P | TIM_CCER_CC##input##NP); \
    const uint32_t rising = ccer & TIM_CCER_CC##input##P;                  \
    if (inverted ? rising : !rising) {                                     \
      name##_length = timer->CCR##input;                                   \
    } else {                                                               \
      timer->EGR = TIM_EGR_UG;                                             \
    }                                                                      \
  }

// You need to enable the clock, set up the alt function for the input pin,
// set the prescaler, and set up the timer input before calling this.
#define timer_setup(timer, irq, input)                           \
  do {                                                           \
    timer->CR1 = TIM_CR1_URS;                                    \
    timer->DIER = TIM_DIER_CC##input##IE;                        \
    timer->CCER = TIM_CCER_CC##input##P | TIM_CCER_CC##input##E; \
    timer->EGR = TIM_EGR_UG;                                     \
    timer->CR1 |= TIM_CR1_CEN;                                   \
    NVIC_SetPriority(irq, 1);                                    \
    NVIC_EnableIRQ(irq);                                         \
  } while (0)

#endif  // CAPE_TIMER_H_
