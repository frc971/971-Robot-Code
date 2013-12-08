#include "cape/uart.h"
#include "cape/uart_common_private.h"

#include "cape/util.h"
#include "cape/uart_common.h"

// TODO(brians): Add error checking.

static void default_callback(int bytes) {}

void uart_transmit_callback(int bytes_transmitted) ALIAS_WEAK(default_callback);
void uart_receive_callback(int bytes_received) ALIAS_WEAK(default_callback);

static int transmit_bytes, receive_bytes;
// These actually contain 1 less than the indicated number to make the common
// path through the ISR faster.
static int transmitted_bytes, received_bytes;
static uint8_t *transmit_data, *receive_data;

// Enable the transmitter and interrupt when we can write.
static const uint32_t kTransmitBits = USART_CR1_TE | USART_CR1_TXEIE;
// Enable the receive and interrupt when there's data to read.
static const uint32_t kReceiveBits = USART_CR1_RE | USART_CR1_RXNEIE;

void USART1_IRQHandler(void) {
  uint32_t status = UART->SR;
  if (status & USART_SR_TXE) {
    if ((transmitted_bytes + 1) < transmit_bytes) {
      UART->DR = transmit_data[++transmitted_bytes];
    } else {
      // Get another interrupt when it's done writing that last byte.
      UART->CR1 = (UART->CR1 & ~USART_CR1_TXEIE) | USART_CR1_TCIE;
    }
  } else if (status & USART_SR_RXNE) {
    receive_data[++received_bytes] = UART->DR;
    if ((received_bytes + 1) >= receive_bytes) {
      UART->CR1 &= ~kReceiveBits;
      uart_receive_callback(receive_bytes);
    }
  } else if (status & USART_SR_TC) {
      UART->CR1 &= ~(USART_CR1_TCIE | USART_CR1_TE);
      uart_transmit_callback(transmit_bytes);
  }
}

void uart_configure(int baud) {
  uart_common_configure(baud);
  NVIC_SetPriority(USART1_IRQn, 3);
  NVIC_EnableIRQ(USART1_IRQn);
}

void uart_transmit(int bytes, uint8_t *data) {
  transmit_bytes = bytes;
  transmitted_bytes = 0;
  transmit_data = data;
  compiler_memory_barrier();
  UART->CR1 |= kTransmitBits;
  UART->DR = data[0];
}

void uart_receive(int bytes, uint8_t *data) {
  receive_bytes = bytes;
  received_bytes = -1;
  receive_data = data;
  compiler_memory_barrier();
  UART->CR1 |= kReceiveBits;
}
