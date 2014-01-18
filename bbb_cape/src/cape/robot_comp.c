#include "cape/robot.h"

#include <STM32F2XX.h>

#include "cape/encoder.h"
#include "cape/analog.h"
#include "cape/digital.h"
#include "cape/util.h"

// TIM11.1 on PB9

static volatile uint32_t ultrasonic_pulse_length = 0;

void TIM1_TRG_COM_TIM11_IRQHandler(void) {
  TIM11->SR = ~TIM_SR_CC1IF;
  ultrasonic_pulse_length = TIM11->CCR1;
}

void robot_init(void) {
  gpio_setup_alt(GPIOB, 11, 3);
  RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
  TIM11->CR1 = TIM_CR1_URS;
  TIM11->SMCR = 5 << 4 /* TI1 */ |
      4 << 0 /* reset and start on edge */;
  TIM11->DIER = TIM_DIER_CC1IE;
  TIM11->CCMR1 = TIM_CCMR1_CC1S_0 /* input pin 1 -> timer input 1 */;
  TIM11->CCER = TIM_CCER_CC1P /* go on falling edge */;
  TIM11->EGR = TIM_EGR_UG;
  TIM11->PSC = 1200 - 1;  // 100kHZ timer
  TIM11->CR1 |= TIM_CR1_CEN;
  NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 3);
  NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
}

void robot_fill_packet(struct DataStruct *packet) {
}
