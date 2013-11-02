#ifndef __ANALOG_H__
#define __ANALOG_H__

#include <stdint.h>

// Internal variable for holding the averaged value. USE analog TO GET TO THIS
// IN CASE IT CHANGES!
uint16_t averaged_values[4];

// Starts the hardware constantly doing conversions on all 4 of our analog
// inputs.
void analog_init(void);

// Retrieves the most recent reading on channel (0-3).
// Returns 0xFFFF for invalid channel.
// 0 means 0V and 0xFFF means 3.3V.
// These values are run through a low-pass filter with unreasonable readings
// discarded first.
uint16_t analog(int channel) {
  if (channel < 0 || channel > 3) return 0xFFFF;
  return averaged_values[channel];
}

#endif  // __ANALOG_H__
