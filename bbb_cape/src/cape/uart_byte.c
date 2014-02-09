#include "cape/uart_byte.h"
#include "cape/uart_common_private.h"

#include <STM32F2XX.h>

#define TIMEOUT_TIM TIM7
#define RCC_APB1ENR_TIMEOUT_TIMEN RCC_APB1ENR_TIM7EN

void uart_byte_configure(void) {
  RCC->APB1ENR |= RCC_APB1ENR_TIMEOUT_TIMEN;

  TIMEOUT_TIM->CR1 = 0;
}

int uart_byte_receive(uint16_t timeout_count, uint16_t timeout_divider) {
  TIMEOUT_TIM->PSC = timeout_divider;
  TIMEOUT_TIM->EGR = TIM_EGR_UG;
  TIMEOUT_TIM->CR1 |= TIM_CR1_CEN;

  while ((UART->SR & USART_SR_RXNE) == 0) {
    if (TIMEOUT_TIM->CNT >= timeout_count) {
      TIMEOUT_TIM->CR1 &= ~TIM_CR1_CEN;
      return -1;
    }
  }

  int r = UART->DR;  // do it now to clear interrupts etc

  if (UART->SR & USART_SR_PE) r = -2;
  if (UART->SR & USART_SR_FE) r = -3;
  if (UART->SR & USART_SR_NE) r = -4;
  if (UART->SR & USART_SR_ORE) r = -5;

  TIMEOUT_TIM->CR1 &= ~TIM_CR1_CEN;
  return r;
}

void uart_byte_send(uint8_t value) {
  while ((UART->SR & USART_SR_TXE) == 0) {}
  UART->DR = value;
}
