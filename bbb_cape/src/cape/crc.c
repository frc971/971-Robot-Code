#include "cape/crc.h"

#include <STM32F2XX.h>

void crc_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
}

uint32_t crc_calculate(uint32_t *data, size_t words) {
  CRC->CR = 1;  // reset it
  uint32_t *end = data + words;
  for (; data < end; ++data) {
    CRC->DR = *data;
  }
  return CRC->DR;
}
