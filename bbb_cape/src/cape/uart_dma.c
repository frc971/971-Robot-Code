#include "cape/uart_dma.h"
#include "cape/uart_common_private.h"

#include "cape/util.h"
#include "cape/uart_common.h"

#define DMA DMA1
#define DMA_STREAM_NUMBER 7
#define DMA_Stream DMA1_Stream7
#define DMA_SR DMA1->HISR
#define DMA_FCR DMA1->HIFCR
#define DMA_SR_SHIFT 3
#define DMA_Stream_IRQHandler DMA1_Stream7_IRQHandler
#define DMA_Stream_IRQn DMA1_Stream7_IRQn
#define RCC_AHB1ENR_DMAEN RCC_AHB1ENR_DMA1EN

#define DMA_SR_BIT(bit) (1 << (bit + 6 * DMA_SR_SHIFT))

void uart_dma_callback(uint8_t *new_buffer) __attribute__((weak));
void uart_dma_callback(uint8_t *new_buffer) {}

static uint8_t *volatile buffer1, *volatile buffer2;

void DMA_Stream_IRQHandler(void) {
  uint32_t status = DMA_SR;
  if (status & DMA_SR_BIT(5)) {  // transfer completed
    DMA_FCR = DMA_SR_BIT(5);
    uart_dma_callback(((DMA_Stream->CR & DMA_SxCR_CT) == 0) ? buffer2
                                                            : buffer1);
  } else if (status & DMA_SR_BIT(3)) {  // transfer error
    DMA_FCR = DMA_SR_BIT(3);
    // Somebody probably wrote to the wrong buffer, which disables the DMA, so
    // we now need to re-enable it.
    // If we're fighting somebody else writing stuff, we'll do this a bunch of
    // times, but oh well.
    DMA_Stream->CR |= DMA_SxCR_EN;
  }
}

void uart_dma_configure(int bytes, uint8_t *buffer1_in, uint8_t *buffer2_in) {
  buffer1 = buffer1_in;
  buffer2 = buffer2_in;
  uart_dma_callback(buffer1);

  RCC->AHB1ENR |= RCC_AHB1ENR_DMAEN;
  DMA_Stream->PAR = (uint32_t)&UART->DR;
  DMA_Stream->M0AR = (uint32_t)buffer1;
  DMA_Stream->M1AR = (uint32_t)buffer2;
  // This is measured in chunks of PSIZE bytes, not MSIZE.
  DMA_Stream->NDTR = bytes;
  DMA_FCR = 0xF << DMA_SR_SHIFT;
  DMA_Stream->CR = DMA_STREAM_NUMBER << 25 |
      DMA_SxCR_DBM /* enable double buffer mode */ |
      2 << 16 /* priority */ |
      2 << 13 /* memory data size = 32 bits */ |
      0 << 11 /* peripherial data size = 8 bits */ |
      DMA_SxCR_MINC /* increment memory address */ |
      1 << 6 /* memory to peripherial */ |
      DMA_SxCR_TCIE | DMA_SxCR_TEIE;
  DMA_Stream->FCR =
      DMA_SxFCR_DMDIS /* disable direct mode (enable the FIFO) */ |
      1 /* 1/2 full threshold */;
  DMA_Stream->CR |= DMA_SxCR_EN;  // enable it
  NVIC_SetPriority(DMA_Stream_IRQn, 6);
  NVIC_EnableIRQ(DMA_Stream_IRQn);

  uart_dma_callback(buffer2);
}
