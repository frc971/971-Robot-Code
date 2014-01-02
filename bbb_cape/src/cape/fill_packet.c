#include "cape/fill_packet.h"

#include <string.h>

#include <STM32F2XX.h>

#include "cape/uart_dma.h"
#include "cape/uart_common.h"
#include "cape/cows.h"
#include "cape/encoder.h"
#include "cape/crc.h"
#include "cape/bootloader_handoff.h"
#include "cape/gyro.h"
#include "cape/analog.h"
#include "cape/robot.h"
#include "cape/digital.h"
#include "cape/led.h"

#include "cape/uart_byte.h"

#define TIMESTAMP_TIM TIM6
#define RCC_APB1ENR_TIMESTAMP_TIMEN RCC_APB1ENR_TIM6EN

static uint8_t buffer1[DATA_STRUCT_SEND_SIZE] __attribute__((aligned(4)));
static uint8_t buffer2[DATA_STRUCT_SEND_SIZE] __attribute__((aligned(4)));

static uint32_t flash_checksum;
// These aren't really integers; they're (4-byte) variables whose addresses mark
// various locations.
extern uint8_t __etext, __data_start__, __data_end__;

static inline void do_fill_packet(struct DataStruct *packet) {
  static uint64_t timestamp = 0;
  counter_update_u64_u16(&timestamp, TIMESTAMP_TIM->CNT);
  packet->timestamp = timestamp;

  packet->flash_checksum = flash_checksum;

  struct GyroOutput gyro_output;
  gyro_get_output(&gyro_output);
  packet->gyro_angle = gyro_output.angle;
  packet->old_gyro_reading = gyro_output.last_reading_bad;
  packet->uninitialized_gyro = !gyro_output.initialized;
  packet->zeroing_gyro = !gyro_output.zeroed;
  packet->bad_gyro = gyro_output.gyro_bad;

  robot_fill_packet(packet);
  //counter_update_u64_u16(&timestamp, TIMESTAMP_TIM->CNT);
  //packet->main.encoders[0] = timestamp;
}

// Fills the new packet with data.
void uart_dma_callback(uint8_t *buffer) {
  struct {
    struct DataStruct packet;
    uint8_t padding[DATA_STRUCT_SEND_SIZE - sizeof(struct DataStruct) - 12];
    uint32_t checksum;
  } __attribute__((packed)) data __attribute__((aligned(4)));
  STATIC_ASSERT(sizeof(data) == DATA_STRUCT_SEND_SIZE - 8,
                The_size_of_the_data_is_wrong);
  struct DataStruct *packet = &data.packet;

  do_fill_packet(packet);

  uint32_t *p;
  memcpy(&p, &packet, sizeof(void *));
  data.checksum = crc_calculate(p, (sizeof(data) - 4) / 4);

  ((uint32_t *)buffer)[0] = 0;
  cows_stuff(&data, sizeof(data), buffer + 4);
}

void fill_packet_start(void) {
  RCC->APB1ENR |= RCC_APB1ENR_TIMESTAMP_TIMEN;
  TIMESTAMP_TIM->CR1 = 0;
  TIMESTAMP_TIM->PSC = 600 - 1;
  TIMESTAMP_TIM->EGR = TIM_EGR_UG;
  TIMESTAMP_TIM->CR1 |= TIM_CR1_CEN;

  crc_init();
  analog_init();
  encoder_init();
  digital_init();

  uint8_t *flash_end = &__etext + (&__data_start__ - &__data_end__) + 8;
  flash_checksum = crc_calculate((void *)MAIN_FLASH_START,
                                 (size_t)(flash_end - MAIN_FLASH_START) / 4);

  led_set(LED_ERR, 0);
  gyro_init();

  uart_common_configure(750000);
  uart_dma_configure(DATA_STRUCT_SEND_SIZE, buffer1, buffer2);
}
