#include "cape/encoder.h"

#include <STM32F2XX.h>

#include "cape/util.h"

// Here is where each encoder is hooked up:
// 0: PC6,PC7 TIM8
// 1: PC0,PC1 EXTI0,EXTI1
// 2: PA0,PA1 TIM5(32)
// 3: PA2,PA3 EXTI2,EXTI3
// 4: PA8,PB0 TIM1
// 5: PA5,PB3 TIM2(32)
// 6: PA6,PB5 TIM3
// 7: PB6,PB7 TIM4

volatile int32_t encoder1_value = 0;
volatile int32_t encoder3_value = 0;

// 1.A
void EXTI0_IRQHandler(void) {
  uint32_t inputs = GPIOA->IDR;
  EXTI->PR = EXTI_PR_PR0;
  // This looks like a weird way to XOR the 2 inputs, but it compiles down to
  // just 2 instructions, which is hard to beat.
  if (((inputs >> 1) ^ inputs) & (1 << 0)) {
    ++encoder1_value;
  } else {
    --encoder1_value;
  }
}

// 1.B
void EXTI1_IRQHandler(void) {
  uint32_t inputs = GPIOA->IDR;
  EXTI->PR = EXTI_PR_PR1;
  if (((inputs >> 1) ^ inputs) & (1 << 0)) {
    --encoder1_value;
  } else {
    ++encoder1_value;
  }
}

// 3.A
void TIM1_TRG_COM_TIM11_IRQHandler(void) {
  uint32_t inputs = GPIOC->IDR;
  TIM9->SR = TIM_SR_CC1IF;
  if (((inputs >> 1) ^ inputs) & (1 << 2)) {
    ++encoder3_value;
  } else {
    --encoder3_value;
  }
}

// 3.B
void EXTI3_IRQHandler(void) {
  uint32_t inputs = GPIOC->IDR;
  EXTI->PR = EXTI_PR_PR3;
  if (((inputs >> 1) ^ inputs) & (1 << 2)) {
    --encoder3_value;
  } else {
    ++encoder3_value;
  }
}

static void encoder_setup(TIM_TypeDef *timer) {
  timer->CR1 = TIM_CR1_UDIS /* wait until we tell it to do anything */ |
               TIM_CR1_URS /* don't generate spurious update interrupts that
                              might be shared with other timers */;
  timer->SMCR = 3;  // 4x quadrature encoder mode
  timer->CCMR1 =
      TIM_CCMR1_CC2S_0 | /* input pin 2 -> timer input 2 */
      TIM_CCMR1_CC1S_0;  /* input pin 1 -> timer input 1*/
  timer->EGR = TIM_EGR_UG;
  timer->CR1 |= TIM_CR1_CEN;
}

void encoder_init(void) {
  // Set up the 3 simple software encoder inputs.
  EXTI_set(0, 2);
  EXTI_set(1, 2);
  EXTI_set(3, 0);
  EXTI->IMR |= EXTI_IMR_MR0 | EXTI_IMR_MR1 | EXTI_IMR_MR3;
  EXTI->RTSR |= EXTI_RTSR_TR0 | EXTI_RTSR_TR1 | EXTI_RTSR_TR3;
  EXTI->FTSR |= EXTI_FTSR_TR0 | EXTI_FTSR_TR1 | EXTI_FTSR_TR3;
  NVIC_EnableIRQ(EXTI0_IRQn);
  NVIC_EnableIRQ(EXTI1_IRQn);
  NVIC_EnableIRQ(EXTI3_IRQn);

  // Set up the A2 software encoder input through TIM9.
  gpio_setup_alt(GPIOA, 2, 3);
  RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
  TIM9->CR1 = TIM_CR1_UDIS;
  TIM9->DIER = TIM_DIER_CC1IE;
  TIM9->CCMR1 = TIM_CCMR1_CC1S_0; /* input pin 1 -> timer input 1 */
  TIM9->CCER = TIM_CCER_CC1NP | TIM_CCER_CC1P | TIM_CCER_CC1E;
  TIM9->EGR = TIM_EGR_UG;
  TIM9->CR1 |= TIM_CR1_CEN;
  NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

  gpio_setup_alt(GPIOA, 8, 1);
  gpio_setup_alt(GPIOB, 0, 1);
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  encoder_setup(TIM1);

  gpio_setup_alt(GPIOA, 5, 1);
  gpio_setup_alt(GPIOB, 3, 1);
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  encoder_setup(TIM2);

  gpio_setup_alt(GPIOA, 6, 2);
  gpio_setup_alt(GPIOB, 5, 2);
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  encoder_setup(TIM3);

  gpio_setup_alt(GPIOB, 6, 2);
  gpio_setup_alt(GPIOB, 7, 2);
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
  encoder_setup(TIM4);

  gpio_setup_alt(GPIOA, 0, 2);
  gpio_setup_alt(GPIOA, 1, 2);
  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
  encoder_setup(TIM5);

  gpio_setup_alt(GPIOC, 6, 3);
  gpio_setup_alt(GPIOC, 7, 3);
  RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
  encoder_setup(TIM8);
}
