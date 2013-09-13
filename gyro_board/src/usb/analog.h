#ifndef __ANALOG_H__
#define __ANALOG_H__

#include <stdint.h>

extern int64_t gyro_angle;

void analog_init(void);
int analog(int channel);

#endif  // __ANALOG_H__
