#ifndef __ANALOG_H__
#define __ANALOG_H__

void analog_init (void);
int analog(int chan);
int digital(int chan);
int encoder_bits(int chan);
void encoder_init(void);
int32_t encoder_val(int chan);
int dip(int chan);
#endif // __ANALOG_H__
