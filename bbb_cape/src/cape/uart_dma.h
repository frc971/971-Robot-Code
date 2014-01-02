#ifndef CAPE_UART_DMA_H_
#define CAPE_UART_DMA_H_

#include <stdint.h>

// This file deals with USART1 over DMA. It sets the DMA stream to double-buffer
// mode and calls a function when it's time to fill a new buffer. It only
// supports sending.

// Callback to be implemented by the user.
// Implemented as a weak symbol that does nothing by default.
// new_buffer is the buffer that should be filled out to be written next.
void uart_dma_callback(uint8_t *new_buffer);

// uart_common_configure must be called before this.
// bytes is the size off buffer1 and buffer2.
// Calls uart_dma_callback twice (for each buffer) to get started.
void uart_dma_configure(int bytes, uint8_t *buffer1, uint8_t *buffer2);

#endif  // CAPE_UART_DMA_H_
