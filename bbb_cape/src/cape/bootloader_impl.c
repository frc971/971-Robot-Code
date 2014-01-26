#include "cape/bootloader_handoff.h"

#include <string.h>

#include "cape/uart_byte.h"
#include "cape/uart_common.h"
#include "cape/crc.h"
#include "cape/util.h"

// The protocol is pretty simple. Basically, when the bootloader is started, it
// expects repeated "packets" of data to write. It starts at MAIN_FLASH_START,
// erases from MAIN_FLASH_START_SECTOR to MAIN_FLASH_END_SECTOR, and keeps
// writing until MAIN_FLASH_END (if it gets data).
//
// The bootloader sends READY when it is first ready to receive bytes. It then
// expects DATA_BYTES-sized packets (+ the checksum calculated with the standard
// CRC algorithm). When it successfully receives one and writes it out, it sends
// ACK. If it has any errors, it waits until there's a 1-second gap (or it
// receives all the bytes and sees a checksum error) and then sends a NACK.

#define DATA_BYTES 256

#define ACK 0x79
#define NACK 0x1F
#define READY 0x7F

static void process_buffer(uint32_t *buffer) {
  static uint32_t *out_pointer = (uint32_t *)MAIN_FLASH_START;
  if ((out_pointer + DATA_BYTES / 4) >= (uint32_t *)MAIN_FLASH_END) return;

  while (FLASH->SR & FLASH_SR_BSY) {}
  FLASH->CR |= FLASH_CR_PG;
  for (int i = 0; i < (DATA_BYTES / 4); ++i) {
    *(out_pointer++) = buffer[i];
  }
  while (FLASH->SR & FLASH_SR_BSY) {}
  FLASH->CR &= ~FLASH_CR_PG;
}

__attribute__((noreturn)) void bootloader_start(void) {
  crc_init();

  // Unlock the flash so we can program it.
  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;
  while (FLASH->CR & FLASH_CR_LOCK) {}

  FLASH->CR =
      (FLASH->CR & ~(FLASH_CR_PSIZE_0 | FLASH_CR_PSIZE_1)) | FLASH_CR_PSIZE_1;

  FLASH->CR |= FLASH_CR_SER;
  for (int i = MAIN_FLASH_START_SECTOR; i <= MAIN_FLASH_END_SECTOR; ++i) {
    while (FLASH->SR & FLASH_SR_BSY) {}
    FLASH->CR = (FLASH->CR & ~(FLASH_CR_SNB_0 | FLASH_CR_SNB_1 |
                               FLASH_CR_SNB_2 | FLASH_CR_SNB_3)) |
                i << 3;
    FLASH->CR |= FLASH_CR_STRT;
    while (FLASH->CR & FLASH_SR_BSY) {}
  }
  FLASH->CR &= ~FLASH_CR_SER;

  uart_common_configure(115200);
  uart_byte_configure();

  uint8_t buffer[DATA_BYTES + 4] __attribute__((aligned(4)));
  // Whether we've already encountered an error in this block or not.
  int error = 0;
  int bytes_received = 0;

  uart_byte_send(READY);

  while (1) {
    // Receive with a 1 second timeout.
    int received = uart_byte_receive(60000, 1000 - 1);
    if (received < 0) {
      if (received == -1) {
        uart_byte_send(NACK);
        error = 0;
        bytes_received = 0;
      } else {
        error = 1;
      }
    } else {  // successfully received a byte
      if (error == 0) {
        buffer[bytes_received++] = (uint8_t)received;
        if (bytes_received == sizeof(buffer)) {
          bytes_received = 0;
          uint32_t checksum;
          memcpy(&checksum, &buffer[DATA_BYTES], 4);
          uint32_t *buffer32;
          uint8_t *buffer8 = buffer;
          memcpy(&buffer32, &buffer8, 4);
          if (crc_calculate(buffer32, DATA_BYTES / 4) != checksum) {
            uart_byte_send(NACK);
          } else {
            process_buffer(buffer32);
            uart_byte_send(ACK);
          }
        }
      }
    }
  }
}
