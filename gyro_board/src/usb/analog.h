#ifndef __ANALOG_H__
#define __ANALOG_H__

#include <stdint.h>

// Starts the hardware constantly doing conversions on all 4 of our analog
// inputs.
void analog_init(void);

// Retrieves the most recent reading on channel (0-3).
// Returns 0xFFFF for invalid channel.
// 0 means 0V and 0x3FF means 3.3V.
uint16_t analog(int channel);

#endif  // __ANALOG_H__
