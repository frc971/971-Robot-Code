#ifndef CAPE_UART_BYTE_H_
#define CAPE_UART_BYTE_H_

#include <stdint.h>

// uart_common_configure must be called before this.
void uart_byte_configure(void);

// Spins until 1 byte is received or some amount of time. The timeout is
// timeout_count*(timeout_divider+1)/60MHz.
// The result is <0 for timeout/error or the received byte.
// errors:
//   -1  timeout
//   -2  parity
//   -3  framing
//   -4  noise detected
//   -5  overrun
int uart_byte_receive(uint16_t timeout_count, uint16_t timeout_divider);

// Spins until 1 byte can be written out.
void uart_byte_send(uint8_t value);

#endif  // CAPE_UART_BYTE_H_
