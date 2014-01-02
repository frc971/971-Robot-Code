#include "cape/uart_common.h"
#include "cape/uart_common_private.h"

#include "cape/util.h"

#define RCC_APB2ENR_UARTEN RCC_APB2ENR_USART1EN

#define FPCLK 60000000

// The UART is on PA9 and PA10.
void uart_common_configure(int baud) {
  gpio_setup_alt(GPIOA, 9, 7);
  gpio_setup_alt(GPIOA, 10, 7);
  RCC->APB2ENR |= RCC_APB2ENR_UARTEN;

  // baud = 60MHz / kMultiplier * (whole_part + fraction / kMultiplier))
  static const int kMultiplier = 16 /* 8 * (2 - OVER8) */;
  // The divisor of FPCLK that we want (*2).
  int divisor = FPCLK * 2 / baud;
  // The whole-number part of the divisor.
  int mantissa = divisor / kMultiplier / 2;
  // The fractional part of the divisor (*2).
  int fraction = divisor % (kMultiplier * 2);
  UART->BRR = (mantissa << 4) | ((fraction + 1) / 2);
  UART->CR1 =
      //USART_CR1_M /* 9th bit for the parity */ |
      //USART_CR1_PCE /* enable parity (even by default) */ |
      //USART_CR1_OVER8 /* support going faster */ |
      0;
  UART->CR1 |= USART_CR1_UE;  // enable it
}
