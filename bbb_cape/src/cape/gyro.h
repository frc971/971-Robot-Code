#ifndef GYRO_BOARD_SRC_USB_GYRO_H_
#define GYRO_BOARD_SRC_USB_GYRO_H_

#include <stdint.h>
#include <string.h>

#include <STM32F2XX.h>

// Does everything to set up the gyro code, including starting a timer which
// triggers reads and integrates the gyro values and blinks the LEDs etc.
void gyro_init(void);

struct GyroOutput {
  int64_t angle;
  int last_reading_bad;
  int gyro_bad;
  int initialized;
  int zeroed;
};

// Reads the most recent output value and avoids race conditions.
// Must be called from a lower-priority ISR than TIM10's.
static inline void gyro_get_output(struct GyroOutput *output) {
  extern struct GyroOutput gyro_output;
  NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
  memcpy(output, &gyro_output, sizeof(gyro_output));
  NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

#endif  // GYRO_BOARD_SRC_USB_GYRO_H_
