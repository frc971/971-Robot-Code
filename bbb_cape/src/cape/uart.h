#ifndef CAPE_UART_H_
#define CAPE_UART_H_

#include <stdint.h>

// This file deals with USART1. It sends bytes from a buffer or receives bytes
// into a buffer and then calls a callback function.

// uart_common_configure must be called before this.
void uart_configure(void);

// Callbacks to be implemented by the user.
// Implemented as weak symbols that do nothing by default.
// The argument is the number of bytes transmitted or received. It will be less
// than the requested number if there was an error.
void uart_transmit_callback(int bytes_transmitted);
void uart_receive_callback(int bytes_received);

void uart_transmit(int bytes, uint8_t *data);
void uart_receive(int bytes, uint8_t *data);

#endif  // CAPE_UART_H_
