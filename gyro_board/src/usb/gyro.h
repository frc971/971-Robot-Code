#ifndef GYRO_BOARD_SRC_USB_GYRO_H_
#define GYRO_BOARD_SRC_USB_GYRO_H_

#include <stdint.h>

// Does everything to set up the gyro code, including starting a task which
// reads and integrates the gyro values and blinks the LEDs etc.
void gyro_init(void);

struct GyroOutput {
  int64_t angle;
  int last_reading_bad;
  int gyro_bad;
  int initialized;
};
// This gets updated in a portENTER_CRITICAL/portEXIT_CRITICAL() block so all of
// the values will be in sync.
extern struct GyroOutput gyro_output;

#endif  // GYRO_BOARD_SRC_USB_GYRO_H_
