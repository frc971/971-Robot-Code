#include "cape/uart_common.h"
#include "cape/uart_common_private.h"

#include "cape/util.h"

#define FPCLK 60000000

// The UART is on PA9 and PA10.
void uart_common_configure(int baud) {
  gpio_setup_alt(GPIOA, 9, 7);
  gpio_setup_alt(GPIOA, 10, 7);
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9;  // we want to go FAST!
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

  // baud = 60MHz / (8 * (2 - OVER8) * (mantissa / fraction))
  int fraction = 8;  // the biggest it can be with OVER8=0
  int mantissa = FPCLK * (16 /* 8 * (2 - OVER8) */ / fraction) / baud;
  UART->BRR = (mantissa << 4) | fraction;
  UART->CR1 = USART_CR1_UE /* enable it */ |
      USART_CR1_M /* 9th bit for the parity */ |
      USART_CR1_PCE /* enable parity (even by default) */;
}
