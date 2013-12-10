#include "cape/fill_packet.h"

#include <string.h>

#include <STM32F2XX.h>

#include "cape/uart_dma.h"
#include "cape/cows.h"

static uint8_t buffer1[DATA_STRUCT_SEND_SIZE] __attribute__((aligned(4)));
static uint8_t buffer2[DATA_STRUCT_SEND_SIZE] __attribute__((aligned(4)));

// Fills the new packet with data.
void uart_dma_callback(uint8_t *buffer) {
  struct {
    struct DataStruct packet;
    uint8_t padding[DATA_STRUCT_SEND_SIZE - sizeof(struct DataStruct) - 12];
    uint32_t checksum;
  } data __attribute__((aligned(4)));
  STATIC_ASSERT(sizeof(data) == DATA_STRUCT_SEND_SIZE - 8,
                The_size_of_the_data_is_wrong);
  struct DataStruct *packet = &data.packet;

  CRC->CR = 1;  // reset it
  uint32_t *p1;
  memcpy(&p1, &packet, sizeof(void *));
  {
    uint32_t *restrict p = p1;
    for (; p < (uint32_t *)(packet + 1); ++p) {
      CRC->DR = *p;
    }
  }
  data.checksum = CRC->DR;

  memset(buffer, 0, 4);
  cows_stuff(&data, sizeof(data), buffer + 4);
}

void fill_packet_start(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;

  uart_dma_configure(3000000, DATA_STRUCT_SEND_SIZE, buffer1, buffer2);
}
