#include "cape/encoder.h"

#include <STM32F2XX.h>

#include "cape/util.h"

// Here is where each encoder is hooked up:
// 0: PC6,PC7 TIM8(APB2)
// 1: PC0,PC1 EXTI0,EXTI1
// 2: PA0,PA1 TIM5(32)
// 3: PA2,PA3 TIM9.1,EXTI3
// 4: PA8,PB0 TIM1.1,TIM3.3
// 5: PA5,PB3 TIM2(32)
// 6: PA6,PB5 TIM3
// 7: PB6,PB7 TIM4

volatile int32_t encoder1_value = 0;
volatile int32_t encoder3_value = 0;
volatile int32_t encoder4_value = 0;

// 1.A
void EXTI0_IRQHandler(void) {
  uint32_t inputs = GPIOC->IDR;
  EXTI->PR = EXTI_PR_PR0;
	int32_t old_value = encoder1_value;
	int32_t new_value;
  // This looks like a weird way to XOR the 2 inputs, but it compiles down to
  // just 2 instructions, which is hard to beat.
  if (((inputs >> 1) ^ inputs) & (1 << 0)) {
		new_value = old_value + 1;
  } else {
		new_value = old_value - 1;
  }
	encoder1_value = new_value;
}

// 1.B
void EXTI1_IRQHandler(void) {
  uint32_t inputs = GPIOC->IDR;
  EXTI->PR = EXTI_PR_PR1;
	int32_t old_value = encoder1_value;
	int32_t new_value;
  if (((inputs >> 1) ^ inputs) & (1 << 0)) {
		new_value = old_value - 1;
  } else {
		new_value = old_value + 1;
  }
	encoder1_value = new_value;
}

// 3.A
void TIM1_BRK_TIM9_IRQHandler(void) {
  uint32_t inputs = GPIOA->IDR;
  TIM9->SR = ~TIM_SR_CC1IF;
	int32_t old_value = encoder3_value;
	int32_t new_value;
  if (((inputs >> 1) ^ inputs) & (1 << 2)) {
		new_value = old_value + 1;
  } else {
		new_value = old_value - 1;
  }
	encoder3_value = new_value;
}

// 3.B
void EXTI3_IRQHandler(void) {
  uint32_t inputs = GPIOA->IDR;
  EXTI->PR = EXTI_PR_PR3;
	int32_t old_value = encoder3_value;
	int32_t new_value;
  if (((inputs >> 1) ^ inputs) & (1 << 2)) {
		new_value = old_value - 1;
  } else {
		new_value = old_value + 1;
  }
	encoder3_value = new_value;
}

#if 0
TODO(brians): Figure out a good way to let robot_comp override this.
// 4.A
void TIM1_CC_IRQHandler(void) {
  uint32_t a_inputs = GPIOA->IDR, b_inputs = GPIOB->IDR;
  TIM1->SR = ~TIM_SR_CC1IF;
	int32_t old_value = encoder4_value;
	int32_t new_value;
  if (((a_inputs >> 8) ^ b_inputs) & (1 << 0)) {
		new_value = old_value + 1;
  } else {
		new_value = old_value - 1;
  }
	encoder4_value = new_value;
}
#endif

// 4.B
void TIM3_IRQHandler(void) {
  uint32_t a_inputs = GPIOA->IDR, b_inputs = GPIOB->IDR;
  TIM3->SR = ~TIM_SR_CC3IF;
	int32_t old_value = encoder4_value;
	int32_t new_value;
  if (((a_inputs >> 8) ^ b_inputs) & (1 << 0)) {
		new_value = old_value - 1;
  } else {
		new_value = old_value + 1;
  }
	encoder4_value = new_value;
}

static void encoder_setup(TIM_TypeDef *timer, int fast) {
  timer->CR1 =
      TIM_CR1_URS | /* don't generate spurious update interrupts that
                     might be shared with other timers */
      (fast ? (1 << 8) : 0) /* divide filter clock by 2 on fast encoders */;
  timer->SMCR = 3;  // 4x quadrature encoder mode
  timer->CCER = 0;
  timer->CCMR1 =
      TIM_CCMR1_CC2S_0 | /* input pin 2 -> timer input 2 */
      TIM_CCMR1_CC1S_0 | /* input pin 1 -> timer input 1 */
      (0xE << 4) | /* divide filter clock by 32, need 6 in a row to trigger */
      (0xE << 12) /* same for other input */;
  timer->PSC = 0;
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

  // Set up the A2 software encoder input through TIM9 input 1.
  gpio_setup_alt(GPIOA, 2, 3);
  RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
  TIM9->CR1 = 0;
  TIM9->DIER = TIM_DIER_CC1IE;
  TIM9->CCMR1 = TIM_CCMR1_CC1S_0; /* input pin 1 -> timer input 1 */
  TIM9->CCER = TIM_CCER_CC1NP | TIM_CCER_CC1P | TIM_CCER_CC1E;
  TIM9->EGR = TIM_EGR_UG;
  TIM9->CR1 |= TIM_CR1_CEN;
  NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

	// Set up the A4 software encoder input through TIM1 input 1.
  gpio_setup_alt(GPIOA, 8, 1);
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	TIM1->CR1 = 0;
	TIM1->DIER = TIM_DIER_CC1IE;
	TIM1->CCMR1 = TIM_CCMR1_CC1S_0; /* input pin 1 -> timer input 1 */
	TIM1->CCER = TIM_CCER_CC1NP | TIM_CCER_CC1P | TIM_CCER_CC1E;
	TIM1->EGR = TIM_EGR_UG;
	TIM1->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM1_CC_IRQn);

	// Set up the B4 software encoder input through TIM3 input 3.
  gpio_setup_alt(GPIOB, 0, 2);
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->CR1 = 0;
	TIM3->DIER = TIM_DIER_CC3IE;
	TIM3->CCMR2 = TIM_CCMR2_CC3S_0; /* input pin 3 -> timer input 3 */
	TIM3->CCER = TIM_CCER_CC3NP | TIM_CCER_CC3P | TIM_CCER_CC3E;
	TIM3->EGR = TIM_EGR_UG;
	TIM3->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM3_IRQn);

  gpio_setup_alt(GPIOA, 5, 1);
  gpio_setup_alt(GPIOB, 3, 1);
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  encoder_setup(TIM2, 0);

  gpio_setup_alt(GPIOA, 6, 2);
  gpio_setup_alt(GPIOB, 5, 2);
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  encoder_setup(TIM3, 0);

  gpio_setup_alt(GPIOB, 6, 2);
  gpio_setup_alt(GPIOB, 7, 2);
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
  encoder_setup(TIM4, 0);

  gpio_setup_alt(GPIOA, 0, 2);
  gpio_setup_alt(GPIOA, 1, 2);
  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
  encoder_setup(TIM5, 0);

  gpio_setup_alt(GPIOC, 6, 3);
  gpio_setup_alt(GPIOC, 7, 3);
  RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
  encoder_setup(TIM8, 1);
}
