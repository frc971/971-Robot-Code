#ifndef GYRO_BOARD_USB_ENCODER_H_
#define GYRO_BOARD_USB_ENCODER_H_

#include <stdint.h>

void encoder_init(void);

// For debugging only.
// Returns the current values of the inputs for the given encoder (as the low 2
// bits).
int encoder_bits(int channel);

// Returns the current position of the given encoder.
int32_t encoder_val(int channel);

#endif  // GYRO_BOARD_USB_ENCODER_H_
