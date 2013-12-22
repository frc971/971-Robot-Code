#include "cape/uart_dma.h"
#include "cape/uart_common_private.h"

#include "cape/util.h"
#include "cape/uart_common.h"
#include "cape/led.h"

#define DMA DMA2
#define DMA_Stream DMA2_Stream7
#define DMA_SR DMA2->HISR
#define DMA_FCR DMA2->HIFCR
#define DMA_Stream_IRQHandler DMA2_Stream7_IRQHandler
#define DMA_Stream_IRQn DMA2_Stream7_IRQn
#define DMA_CHANNEL_NUMBER 4
#define RCC_AHB1ENR_DMAEN RCC_AHB1ENR_DMA2EN

#define DMA_SR_SHIFT(value) ((value) << 22)
#define DMA_SR_BIT(bit) DMA_SR_SHIFT(1 << (bit))

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
    led_set(LED_ERR, 1);
  }
}

void uart_dma_configure(int bytes, uint8_t *buffer1_in, uint8_t *buffer2_in) {
  buffer1 = buffer1_in;
  buffer2 = buffer2_in;
  uart_dma_callback(buffer1);

  UART->CR3 = USART_CR3_DMAT;
  UART->CR1 |= USART_CR1_TE;

  RCC->AHB1ENR |= RCC_AHB1ENR_DMAEN;
  DMA_Stream->CR = 0;
  while (DMA_Stream->CR & DMA_SxCR_EN);  // make sure it's disabled
  DMA_Stream->PAR = (uint32_t)&UART->DR;
  DMA_Stream->M0AR = (uint32_t)buffer1;
  DMA_Stream->M1AR = (uint32_t)buffer2;
  // This is measured in chunks of PSIZE bytes, not MSIZE.
  DMA_Stream->NDTR = bytes;
  DMA_Stream->CR = DMA_CHANNEL_NUMBER << 25 |
      DMA_SxCR_DBM /* enable double buffer mode */ |
      2 << 16 /* priority */ |
      2 << 13 /* memory data size = 32 bits */ |
      0 << 11 /* peripherial data size = 8 bits */ |
      DMA_SxCR_MINC /* increment memory address */ |
      1 << 6 /* memory to peripherial */ |
      //DMA_SxCR_PFCTRL /* peripherial controls flow */ |
      DMA_SxCR_TCIE | DMA_SxCR_TEIE;
  DMA_Stream->FCR =
      DMA_SxFCR_DMDIS /* disable direct mode (enable the FIFO) */ |
      1 /* 1/2 full threshold */;
  UART->SR = ~USART_SR_TC;
  DMA_FCR = DMA_SR_SHIFT(1 << 0 | 1 << 2 | 1 << 3 | 1 << 4 | 1 << 5);
  DMA_Stream->CR |= DMA_SxCR_EN;  // enable it
  NVIC_SetPriority(DMA_Stream_IRQn, 8);
  NVIC_EnableIRQ(DMA_Stream_IRQn);

  uart_dma_callback(buffer2);
}
